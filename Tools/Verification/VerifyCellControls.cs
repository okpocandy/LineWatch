using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

public static class VerifyCellControls
{
    private static readonly string[] DataSources = {
        "Assets/01.Scenes/LineWatch.unity", "Assets/01.Scenes/RobotLab.unity",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab",
        "Assets/03.Prefabs/GripTrialPart.prefab", "Assets/03.Prefabs/HandoffTrialPart.prefab",
        "Assets/04.ScriptableObject/ScaraCellHandoff.asset", "Assets/04.ScriptableObject/FanucCellHandoff.asset",
        "Assets/04.ScriptableObject/ScaraLabWaypoints.asset", "Assets/04.ScriptableObject/FanucLabWaypoints.asset",
        "ProjectSettings/ProjectSettings.asset", "ProjectSettings/EditorBuildSettings.asset"
    };

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>(); Require(layout != null, "Open LineWatch.");
        foreach (var g in new[] { layout.Scara.GetComponent<RobotGripper>(), layout.Fanuc.GetComponent<RobotGripper>() })
        {
            var root = Window(g);
            Require(!root.Q<Button>("gripper-prepare").enabledSelf && !root.Q<Button>("gripper-cycle-start").enabledSelf &&
                !root.Q<Button>("gripper-cycle-resume").enabledSelf, "Edit Mode UI not locked.");
        }
        var legacy = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/03.Prefabs/GripTrialPart.prefab").GetComponent<GrippableItem>();
        Require(legacy.GetGripPoint(RobotWaypointSet.RobotType.Scara) == legacy.GripPoint &&
            legacy.GetGripPoint(RobotWaypointSet.RobotType.Fanuc) == legacy.GripPoint, "Legacy frame mapping changed.");
        var report = new { passed = true, editPanelsLocked = 2, legacyFrameMappingPassed = true, directButtonClicks = 0 };
        Write("cell-controls-edit.json", report); return JsonConvert.SerializeObject(report);
    }

    public static async Task<object> Play()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh unpaused LineWatch Play Mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch.");
        var robots = new[] { layout.Scara.GetComponent<RobotGripper>(), layout.Fanuc.GetComponent<RobotGripper>() };
        var labs = robots.Select(g => g.GetComponent<RobotGripperLab>()).ToArray();
        var sequences = robots.Select(g => g.GetComponent<RobotWaypointSequence>()).ToArray();
        Require(labs.All(l => l.TrialItem == null) && sequences.All(s => s.State == RobotWaypointSequence.RunState.Idle), "Fresh fixtures required.");
        var sources = DataSources.Concat(Directory.GetFiles("Assets/02.Scripts/LineWatch", "*.cs", SearchOption.AllDirectories)).Distinct().ToDictionary(p => p, Hash);
        var sample = layout.GetComponentsInChildren<GrippableItem>().Single(); var samplePosition = sample.Body.position; var sampleRotation = sample.Body.rotation;
        float scale = Time.timeScale, maxPosition = 0, maxAngle = 0;
        int axisRoundTrips = 0, stoppedMoves = 0, arrivals = 0, manualGrips = 0, cycles = 0, heldPauses = 0, rejected = 0, uiChecks = 0;
        var rows = new List<object>();
        void Reject(int r, Func<bool> request, string name)
        {
            var item = labs[r].TrialItem; var p = item != null ? item.Body.position : Vector3.zero;
            var q = item != null ? item.Body.rotation : Quaternion.identity; var owner = item != null ? item.Owner : null;
            var before = Joints(robots[r]); uint version = Version(robots[r]);
            Require(!request() && Version(robots[r]) == version && before.SequenceEqual(Joints(robots[r])) &&
                (item == null || item.Body.position.Equals(p) && item.Body.rotation.Equals(q) && item.Owner == owner), "Rejected request mutated state: " + name);
            rejected++;
        }
        void Measure(RobotGripper g, Pose expected, string name)
        {
            float p = Vector3.Distance(g.Contact.position, expected.position), a = RobotGripper.RotationError(g.Contact.rotation, expected.rotation);
            Require(p <= .002f && a <= .5f, "World contact missed target: " + name);
            maxPosition = Mathf.Max(maxPosition, p); maxAngle = Mathf.Max(maxAngle, a);
            rows.Add(new { name, positionErrorMetres = p, rotationErrorDegrees = a });
        }
        async Task Display(int r)
        {
            var root = Window(robots[r]); await Task.Delay(160);
            var item = labs[r].TrialItem; Require(item.TryGetGripPose(Kind(robots[r]), out var pose), "Selected frame unavailable.");
            string expected = $"기준 오차: {Vector3.Distance(robots[r].Contact.position, pose.position) * 1000:F3}mm / {RobotGripper.RotationError(robots[r].Contact.rotation, pose.rotation):F3}°";
            Require(root.Q<Label>("gripper-alignment").text == expected && root.Q<ObjectField>("gripper-item").value == item, "Panel bound to wrong frame/item.");
            uiChecks++;
        }
        try
        {
            Time.timeScale = 10;
            for (int r = 0; r < 2; r++)
            {
                var g = robots[r]; var other = Joints(robots[1 - r]);
                int count = r == 0 ? 4 : 6;
                for (int axis = 0; axis < count; axis++)
                {
                    float target = r == 0 && axis == 2 ? .04f : 10f;
                    Require(MoveJoint(g, axis, target), "Manual joint request failed."); await Until(() => Rest(g));
                    Require(Mathf.Abs(Joints(g)[axis] - target) < (r == 0 && axis == 2 ? .000001f : .001f), "Joint target missed.");
                    Home(g); await Until(() => Rest(g)); Require(Joints(g).All(q => Mathf.Abs(q) < .00001f), "Home failed."); axisRoundTrips++;
                }
                Require(MoveJoint(g, 0, 30), "Stop fixture rejected.");
                await Until(() => Joints(g)[0] > 1); Require(!Rest(g) && Joints(g)[0] < 30, "Stop fixture reached endpoint too early.");
                g.Stop(); await Ticks(2); var stopped = Joints(g); await Ticks(4);
                Require(Rest(g) && stopped.SequenceEqual(Joints(g)), "Manual stop drifted."); stoppedMoves++;
                Home(g); await Until(() => Rest(g));
                var stations = r == 0 ? new[] { layout.InputStop, layout.HandoffPoint } : new[] { layout.HandoffPoint, layout.OutputA, layout.OutputB };
                foreach (var station in stations) foreach (float height in new[] { r == 0 ? .07f : .06f, 0f })
                {
                    var product = new Pose(station.position + Vector3.up * height, station.rotation);
                    var tcp = ProductTcp(g, layout.ProductPrefab.GetGripPoint(Kind(g)), product);
                    MoveTcp(g, tcp); await Until(() => Rest(g));
                    var point = layout.ProductPrefab.GetGripPoint(Kind(g));
                    Measure(g, new Pose(product.position + product.rotation * point.localPosition, product.rotation * point.localRotation), station.name + (height == 0 ? " work" : " approach")); arrivals++;
                }
                Home(g); await Until(() => Rest(g));
                var lab = labs[r]; Require(lab.TryPrepare(out var e), e); await Ticks(2);
                Require(lab.TryMoveToWork(out e), e); await Until(() => Rest(g)); await Display(r);
                var item = lab.TrialItem; var original = item.Body.position;
                item.Body.position += Vector3.up * .01f; Reject(r, () => g.TryGrip(item, out _), "10mm physical mismatch"); item.Body.position = original;
                Time.timeScale = 0; var panel = Window(g); await Task.Delay(160);
                Require(!panel.Q<Button>("gripper-cycle-start").enabledSelf && !panel.Q<Button>("gripper-grip").enabledSelf, "Paused clock UI enabled."); uiChecks++;
                Reject(r, () => g.TryGrip(item, out _), "paused clock grip"); Time.timeScale = 10;
                var before = item.Body.position; var beforeR = item.Body.rotation;
                Require(g.TryGrip(item, out e), e); Require(item.Owner == g && before.Equals(item.Body.position) && beforeR.Equals(item.Body.rotation), "Manual claim snapped item.");
                Require(lab.TryLift(out e), e); Reject(r, () => g.TryRelease(out _), "release during lift");
                await Until(() => Vector3.Distance(g.Contact.position, before) > .001f);
                g.Stop(); await Ticks(2); var hold = item.Body.position; var holdRotation = item.Body.rotation; await Ticks(4);
                Require(Rest(g) && item.Owner == g && g.HeldItem == item && hold.Equals(item.Body.position) && holdRotation.Equals(item.Body.rotation), "Held stop drifted/lost item."); heldPauses++;
                Require(lab.TryLift(out e), e); await Until(() => Rest(g)); await Ticks(2);
                Require(item.TryGetGripPose(Kind(g), out var selected), "Selected grip lost."); Measure(g, selected, "manual held frame");
                Require(g.TryRelease(out e), e); var released = item.Body.position; var releasedR = item.Body.rotation; await Ticks(4);
                Require(item.Owner == null && released.Equals(item.Body.position) && releasedR.Equals(item.Body.rotation), "Manual release drifted."); manualGrips++;
                Home(g); await Until(() => Rest(g));
                for (int run = 0; run < 2; run++)
                {
                    Require(lab.TryPrepare(out e), e); await Ticks(2);
                    Require(lab.TryStartPickPlace(out e), e);
                    Reject(r, () => lab.TryStartPickPlace(out _), "repeat auto start");
                    var sequence = sequences[r];
                    if (run == 1)
                    {
                        await Until(() => sequence.StepIndex == 2 && sequence.GripCount == 1 || sequence.State == RobotWaypointSequence.RunState.Faulted || sequence.State == RobotWaypointSequence.RunState.Completed);
                        Require(sequence.State == RobotWaypointSequence.RunState.Running && sequence.StepIndex == 2 && g.HeldItem == item, "Missed carrying stage: " + sequence.LastError);
                        sequence.Pause(); await Ticks(2); hold = item.Body.position; holdRotation = item.Body.rotation; await Ticks(4);
                        Require(Rest(g) && item.Owner == g && hold.Equals(item.Body.position) && holdRotation.Equals(item.Body.rotation), "Auto pause drifted/lost item."); heldPauses++;
                        Reject(r, () => lab.TryPrepare(out _), "prepare while held");
                        panel = Window(g); await Task.Delay(160);
                        Require(panel.Q<Button>("gripper-cycle-resume").enabledSelf && !panel.Q<Button>("gripper-cycle-start").enabledSelf && !panel.Q<Button>("gripper-prepare").enabledSelf, "Paused held UI inconsistent."); uiChecks++;
                        Require(sequence.TryResume(out e), e);
                    }
                    await Until(() => sequence.State == RobotWaypointSequence.RunState.Completed || sequence.State == RobotWaypointSequence.RunState.Faulted);
                    Require(sequence.State == RobotWaypointSequence.RunState.Completed && sequence.GripCount == 1 && sequence.ReleaseCount == 1 && item.Owner == null && g.HeldItem == null && Rest(g), "Auto cycle failed/repeated: " + sequence.LastError);
                    var goal = sequence.Waypoints.Retreat; var rotation = g.RobotFrame.rotation * goal.Rotation;
                    var expected = new Pose(g.RobotFrame.TransformPoint(goal.PositionMetres) + rotation * g.Tcp.InverseTransformPoint(g.Contact.position),
                        rotation * (Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation));
                    Require(item.TryGetGripPose(Kind(g), out selected), "Released selected frame missing.");
                    float pe = Vector3.Distance(expected.position, selected.position), re = RobotGripper.RotationError(expected.rotation, selected.rotation);
                    Require(pe <= .002f && re <= .5f, "Auto released product missed selected frame.");
                    maxPosition = Mathf.Max(maxPosition, pe); maxAngle = Mathf.Max(maxAngle, re);
                    released = item.Body.position; releasedR = item.Body.rotation; await Ticks(4);
                    Require(released.Equals(item.Body.position) && releasedR.Equals(item.Body.rotation), "Auto release drifted.");
                    await Display(r); cycles++;
                }
                Require(other.SequenceEqual(Joints(robots[1 - r])), "Other robot moved during single-robot test.");
                Home(g); await Until(() => Rest(g));
            }
            Require(sample.Owner == null && samplePosition.Equals(sample.Body.position) && sampleRotation.Equals(sample.Body.rotation), "Infeed sample changed.");
            Require(sources.All(p => Hash(p.Key) == p.Value), "Source changed during verification.");
            var report = new { passed = true, axisRoundTrips, stoppedMoves, stationArrivals = arrivals, manualGripRuns = manualGrips,
                automaticCycles = cycles, heldPauses, rejectedRequestsPreserved = rejected, uiReadChecks = uiChecks,
                maxWorldPositionErrorMetres = maxPosition, maxWorldRotationErrorDegrees = maxAngle,
                sourceFiles = sources.Count, sourceAssetsUnchanged = true, sources, rows, directButtonClicks = 0,
                scope = "10x independent robot API/control and selected-frame panel verification in LineWatch; no shared-item transfer, cell interlock, collision/force or production flow validation" };
            Write("cell-controls-play.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.axisRoundTrips, report.stoppedMoves, report.stationArrivals, report.manualGripRuns,
                report.automaticCycles, report.heldPauses, report.rejectedRequestsPreserved, report.uiReadChecks, report.maxWorldPositionErrorMetres,
                report.maxWorldRotationErrorDegrees, report.sourceFiles, report.sourceAssetsUnchanged });
        }
        finally
        {
            Time.timeScale = scale;
            for (int r = 0; r < 2; r++)
            {
                sequences[r].Pause(); robots[r].Stop(); if (robots[r].HeldItem != null) robots[r].TryRelease(out _);
                if (labs[r].TrialItem != null) UnityEngine.Object.Destroy(labs[r].TrialItem.gameObject);
            }
        }
    }

    public static Task<object> ProbeAlignment() => Alignment(false);
    public static Task<object> CheckAlignment() => Alignment(true);

    private static async Task<object> Alignment(bool fixedExpected)
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play Mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch.");
        var lab = layout.Fanuc.GetComponent<RobotGripperLab>();
        Require(lab.TrialItem == null, "Fresh trial required.");
        float scale = Time.timeScale;
        try
        {
            Time.timeScale = 10;
            Require(lab.TryPrepare(out var e), e); Require(lab.TryMoveToWork(out e), e);
            await Until(() => Rest(lab.Gripper));
            Require(EditorApplication.ExecuteMenuItem("LineWatch/FANUC 조작 테스트"), "FANUC menu missing.");
            var root = EditorWindow.GetWindow<FanucJointLabWindow>().rootVisualElement;
            await Task.Delay(250);
            var item = lab.TrialItem; var g = lab.Gripper;
            Require(item.TryGetGripPose(RobotWaypointSet.RobotType.Fanuc, out var selected), "Selected frame missing.");
            float p = Vector3.Distance(g.Contact.position, selected.position), a = RobotGripper.RotationError(g.Contact.rotation, selected.rotation);
            float legacyAngle = RobotGripper.RotationError(g.Contact.rotation, item.GripPose.rotation);
            string expected = $"기준 오차: {p * 1000:F3}mm / {a:F3}°";
            string text = root.Q<Label>("gripper-alignment").text;
            Require(p < .002f && a < .5f && legacyAngle > 80, "Fixture must distinguish selected and legacy frames.");
            Require((text == expected) == fixedExpected, "Unexpected alignment label: " + text + "; expected " + expected);
            var report = new { passed = true, problemReproduced = !fixedExpected, panelMatchesSelectedFrame = text == expected,
                selectedPositionErrorMetres = p, selectedRotationErrorDegrees = a, legacyRotationErrorDegrees = legacyAngle,
                displayed = text, expected, directButtonClicks = 0 };
            Write(fixedExpected ? "cell-alignment-fixed.json" : "cell-alignment-before.json", report);
            return JsonConvert.SerializeObject(report);
        }
        finally { lab.Gripper.Stop(); if (lab.TrialItem != null) UnityEngine.Object.Destroy(lab.TrialItem.gameObject); Time.timeScale = scale; }
    }

    private static bool Rest(RobotGripper g) => g.Scara != null
        ? !g.Scara.IsTrajectoryMoving && !g.Scara.Axis1.IsMoving && !g.Scara.Axis2.IsMoving && !g.Scara.Axis3.IsMoving && !g.Scara.Axis4.IsMoving
        : !g.Fanuc.IsTcpMoving && g.Fanuc.Joints.All(j => !j.IsMoving);
    private static float[] Joints(RobotGripper g) => g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition }
        : g.Fanuc.Joints.Select(j => j.CurrentPosition).ToArray();
    private static uint Version(RobotGripper g) => g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion;
    private static bool MoveJoint(RobotGripper g, int index, float value) => g.Scara != null ? g.Scara.MoveJoint(index, value) : g.Fanuc.MoveJoint(index, value);
    private static void Home(RobotGripper g) { if (g.Scara != null) g.Scara.HomeAll(); else g.Fanuc.HomeAll(); }
    private static RobotWaypointSet.RobotType Kind(RobotGripper g) => g.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc;
    private static Pose ProductTcp(RobotGripper g, Transform point, Pose product)
    {
        var rotation = product.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = product.position + product.rotation * point.localPosition - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        return new Pose(g.RobotFrame.InverseTransformPoint(position), Quaternion.Inverse(g.RobotFrame.rotation) * rotation);
    }
    private static void MoveTcp(RobotGripper g, Pose tcp)
    {
        string error;
        bool accepted = g.Scara != null ? g.Scara.TryMoveTcp(tcp.position, Vector3.SignedAngle(Vector3.forward, tcp.rotation * Vector3.forward, Vector3.up),
            ScaraJointLab.IkSelection.PositiveElbow, ScaraJointLab.MotionMode.Joint, out _, out error)
            : g.Fanuc.TryMoveTcp(tcp.position, tcp.rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error);
        Require(accepted, error);
    }
    private static VisualElement Window(RobotGripper g)
    {
        Require(EditorApplication.ExecuteMenuItem(g.Scara != null ? "LineWatch/SCARA 4축 조작 테스트" : "LineWatch/FANUC 조작 테스트"), "Menu missing.");
        return g.Scara != null ? EditorWindow.GetWindow<ScaraJointLabWindow>().rootVisualElement : EditorWindow.GetWindow<FanucJointLabWindow>().rootVisualElement;
    }
    private static async Task Ticks(int count)
    { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= count * Time.fixedDeltaTime); }
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static async Task Until(Func<bool> condition)
    { var watch = Stopwatch.StartNew(); while (!condition()) { Require(watch.Elapsed.TotalSeconds < 15, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string file, object data) { Directory.CreateDirectory("Logs/LW09"); File.WriteAllText("Logs/LW09/" + file, JsonConvert.SerializeObject(data, Formatting.Indented)); }
    private static void Require(bool value, string message) { if (!value) throw new InvalidOperationException(message); }
}
