using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using State = LineWatch.RobotWaypointSequence.RunState;

public static class VerifyRobotPickPlace
{
    private static readonly string[] Sources = {
        "Assets/01.Scenes/RobotLab.unity", "ProjectSettings/ProjectSettings.asset",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab", "Assets/03.Prefabs/GripTrialPart.prefab",
        "Assets/04.ScriptableObject/ScaraLabWaypoints.asset", "Assets/04.ScriptableObject/FanucLabWaypoints.asset",
        "Assets/02.Scripts/LineWatch/RobotWaypointSequence.cs", "Assets/02.Scripts/LineWatch/RobotGripper.cs", "Assets/02.Scripts/LineWatch/RobotGripperLab.cs" };

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        var labs = UnityEngine.Object.FindObjectsByType<RobotGripperLab>(FindObjectsSortMode.None).OrderBy(l => l.Gripper.Fanuc != null).ToArray();
        Require(labs.Length == 2 && labs.All(l => l.IsReady), "Two ready fixtures required.");
        foreach (var lab in labs) Require(!lab.TryStartPickPlace(out _), "Edit Mode cycle accepted.");
        OpenMenus();
        foreach (var root in Roots()) Require(!root.Q<Button>("gripper-cycle-start").enabledSelf && !root.Q<Button>("gripper-cycle-resume").enabledSelf, "Edit cycle UI not locked.");
        var report = new { passed = true, robots = 2, editRejected = true, uiLocked = true, directButtonClicks = 0 };
        Write("pick-place-edit-verification.json", report); return report;
    }

    public static async Task<object> Play()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use unpaused fresh Play Mode.");
        var labs = UnityEngine.Object.FindObjectsByType<RobotGripperLab>(FindObjectsSortMode.None).OrderBy(l => l.Gripper.Fanuc != null).ToArray();
        Require(labs.Length == 2 && labs.All(l => l.IsReady && l.TrialItem == null), "Fresh fixtures required.");
        var sequences = labs.Select(l => l.GetComponent<RobotWaypointSequence>()).ToArray();
        Require(sequences.All(s => s.State == State.Idle), "No automatic Play startup allowed.");
        var hashes = Sources.ToDictionary(p => p, Hash); var token = labs[0].destroyCancellationToken;
        float originalScale = Time.timeScale;
        int cycles = 0, rejected = 0, heldPauses = 0, emptyPauses = 0, faults = 0, manualOverrides = 0;
        float maxPlacedPosition = 0, maxPlacedRotation = 0;
        var rows = new List<object>();
        bool AtRest(RobotGripper g) => g.Scara != null ? !g.Scara.IsTrajectoryMoving && !g.Scara.Axis1.IsMoving && !g.Scara.Axis2.IsMoving && !g.Scara.Axis3.IsMoving && !g.Scara.Axis4.IsMoving
            : !g.Fanuc.IsTcpMoving && g.Fanuc.Joints.All(j => !j.IsMoving);
        string Snapshot(int r)
        {
            var s = sequences[r]; var g = labs[r].Gripper; var item = labs[r].TrialItem;
            return JsonConvert.SerializeObject(new { s.State, s.StepIndex, s.GripCount, s.ReleaseCount,
                version = g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion,
                holder = g.HeldItem != null ? g.HeldItem.GetInstanceID() : 0,
                part = item != null ? new { owner = item.Owner != null ? item.Owner.GetInstanceID() : 0, p = V(item.Body.position), q = Q(item.Body.rotation) } : null });
        }
        void Reject(int r, Func<bool> request, string name)
        { string before = Snapshot(r); Require(!request() && before == Snapshot(r), "Rejected request mutated state: " + name); rejected++; }
        async Task Ticks(int count)
        { double t = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - t >= count * Time.fixedDeltaTime, token); }
        async Task Prepare(int r)
        {
            var s = sequences[r]; var g = labs[r].Gripper;
            s.Pause(); g.Stop(); await Until(() => AtRest(g), token);
            Require(g.HeldItem == null && labs[r].TryPrepare(out _), "Cannot reset an unheld trial."); await Ticks(2);
        }
        async Task HeldStage(int r)
        {
            await Until(() => sequences[r].State == State.Faulted || sequences[r].StepIndex == 2 && sequences[r].GripCount == 1, token);
            Require(sequences[r].State == State.Running && labs[r].Gripper.HeldItem == labs[r].TrialItem, "Failed before carrying: " + sequences[r].LastError);
        }
        async Task CheckCompletion(int r, string scenario)
        {
            var s = sequences[r]; var g = labs[r].Gripper; var item = labs[r].TrialItem;
            await Until(() => s.State == State.Completed || s.State == State.Faulted, token);
            Require(s.State == State.Completed, "Cycle faulted: " + scenario + " " + s.LastError);
            Require(s.IsPickPlace && s.StepIndex == 4 && s.GripCount == 1 && s.ReleaseCount == 1 && g.HeldItem == null && item.Owner == null && AtRest(g), "Actions repeated/skipped or ownership remained.");
            var goal = s.Waypoints.Retreat; var rotation = g.RobotFrame.rotation * goal.Rotation;
            var expected = new Pose(g.RobotFrame.TransformPoint(goal.PositionMetres) + rotation * g.Tcp.InverseTransformPoint(g.Contact.position),
                rotation * (Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation));
            var actual = item.GripPose;
            float pe = Vector3.Distance(actual.position, expected.position), re = RobotGripper.RotationError(actual.rotation, expected.rotation);
            Require(pe <= .002f && re <= .5f && s.LastReleasePose.position.Equals(item.Body.position), "Released part missed its transport destination.");
            maxPlacedPosition = Mathf.Max(maxPlacedPosition, pe); maxPlacedRotation = Mathf.Max(maxPlacedRotation, re);
            var released = item.Body.position; await Ticks(4);
            Require(released.Equals(item.Body.position), "Released part moved after completion.");
            rows.Add(new { robot = r, scenario, grips = s.GripCount, releases = s.ReleaseCount, actualGripPoint = V(actual.position), target = V(expected.position), pe, re }); cycles++;
        }
        try
        {
            Time.timeScale = 10;
            for (int r = 0; r < 2; r++)
            {
                var lab = labs[r]; var g = lab.Gripper; var s = sequences[r];
                Reject(r, () => lab.TryStartPickPlace(out _), "missing part");
                for (int mode = 0; mode < 3; mode++)
                {
                    await Prepare(r); var item = lab.TrialItem; var before = item.Body.position;
                    if (mode == 0)
                    {
                        string mismatch = null;
                        Reject(r, () => s.TryStartPickPlace(labs[1 - r].Gripper, item, out mismatch), "wrong robot gripper");
                        Require(mismatch.Contains("같은 로봇"), "Wrong robot was rejected for an unrelated reason.");
                    }
                    Require(lab.TryStartPickPlace(out var error), error);
                    Reject(r, () => lab.TryStartPickPlace(out _), "repeat running start");
                    if (mode == 1)
                    {
                        Require(s.StepIndex == 0 && s.GripCount == 0, "Approach fixture skipped."); s.Pause();
                        await Ticks(4); Require(s.StepIndex == 0 && s.GripCount == 0 && item.Owner == null && before.Equals(item.Body.position), "Pre-grip pause changed the part."); emptyPauses++;
                        Require(s.TryResume(out error), error);
                    }
                    if (mode == 2)
                    {
                        await HeldStage(r); int heldStep = s.StepIndex;
                        Reject(r, () => g.TryRelease(out _), "manual release during automatic motion");
                        Reject(r, () => lab.TryMoveToWork(out _), "manual fixture move during automatic motion");
                        if (r == 0) s.Pause(); else g.Stop();
                        await Until(() => s.State == State.Paused, token); await Ticks(2); var held = item.Body.position;
                        await Ticks(4); Require(g.HeldItem == item && item.Owner == g && held.Equals(item.Body.position) && s.StepIndex == heldStep && s.GripCount == 1 && s.ReleaseCount == 0, "Pause dropped or repeated grip."); heldPauses++;
                        Reject(r, () => s.TryStart(out _), "restart while holding");
                        Reject(r, () => lab.TryPrepare(out _), "reset while holding");
                        EditorApplication.isPaused = true; Reject(r, () => s.TryResume(out _), "editor-paused resume"); EditorApplication.isPaused = false;
                        Time.timeScale = 0; Reject(r, () => s.TryResume(out _), "zero-time resume"); Time.timeScale = 10;
                        Require(s.TryResume(out error), error);
                    }
                    await CheckCompletion(r, mode == 0 ? "normal" : mode == 1 ? "pause before grip" : "pause while holding");
                    Require(Vector3.Distance(before, item.Body.position) > .01f, "Cycle did not transport the part.");
                }

                await Prepare(r); var target = lab.TrialItem; var workPosition = target.Body.position;
                target.Body.position = workPosition + Vector3.right * .01f;
                Reject(r, () => lab.TryStartPickPlace(out _), "misplaced part preflight"); target.Body.position = workPosition;
                Require(lab.TryStartPickPlace(out var e), e);
                target.Body.position = workPosition + Vector3.right * .01f;
                await Until(() => s.State == State.Faulted, token);
                Require(s.StepIndex == 1 && s.GripCount == 0 && s.ReleaseCount == 0 && g.HeldItem == null && AtRest(g), "Grip failure advanced or acquired part."); faults++;
                Reject(r, () => s.TryResume(out _), "resume with misplaced part");
                Require(lab.TryPrepare(out e) && s.TryResume(out e), e);
                await CheckCompletion(r, "restore moved part and resume");

                await Prepare(r); Require(lab.TryStartPickPlace(out e), e); await HeldStage(r); s.Pause();
                Require(g.TryRelease(out e), e); Reject(r, () => s.TryResume(out _), "manual release invalidates carry");
                Require(lab.TryPrepare(out e) && lab.TryStartPickPlace(out e), e);
                await CheckCompletion(r, "fresh restart after manual release");

                await Prepare(r); Require(lab.TryStartPickPlace(out e), e); await HeldStage(r);
                var lost = lab.TrialItem; lost.gameObject.SetActive(false);
                await Until(() => s.State == State.Faulted, token);
                Require(s.StepIndex == 2 && s.GripCount == 1 && s.ReleaseCount == 0 && g.HeldItem == null && lost.Owner == null && AtRest(g), "Part loss did not stop carrying."); faults++;
                Reject(r, () => s.TryResume(out _), "lost part resume"); lost.gameObject.SetActive(true);
                Reject(r, () => s.TryResume(out _), "reactivation does not regrip");
                Require(lab.TryPrepare(out e) && lab.TryStartPickPlace(out e), e);
                await CheckCompletion(r, "fresh restart after part loss");

                await Prepare(r); Require(lab.TryStartPickPlace(out e), e); await HeldStage(r);
                if (g.Scara != null) Require(g.Scara.MoveJoint(0, -20), "Manual move rejected."); else Require(g.Fanuc.MoveJoint(0, -20), "Manual move rejected.");
                await Until(() => s.State == State.Paused, token);
                Require((g.Scara != null ? g.Scara.Axis1.TargetPosition : g.Fanuc.Joints[0].TargetPosition) == -20 && g.HeldItem == lab.TrialItem, "Manual command overwritten or lost grip.");
                Reject(r, () => s.TryResume(out _), "manual motion still active");
                await Until(() => AtRest(g), token); Require(s.TryResume(out e), e);
                await CheckCompletion(r, "manual motion takeover and resume"); manualOverrides++;

                var originalData = s.Waypoints; var copy = UnityEngine.Object.Instantiate(originalData);
                SetWaypoints(s, copy);
                try
                {
                    await Prepare(r); Require(lab.TryStartPickPlace(out e), e); await HeldStage(r);
                    var data = new SerializedObject(copy); var retreat = copy.Retreat.PositionMetres;
                    data.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = retreat + Vector3.right * .01f;
                    data.ApplyModifiedPropertiesWithoutUndo();
                    await Until(() => s.State == State.Faulted, token);
                    Require(s.StepIndex == 2 && s.GripCount == 1 && s.ReleaseCount == 0 && g.HeldItem == lab.TrialItem && AtRest(g), "Changed program lost the held part/progress."); faults++;
                    Reject(r, () => s.TryResume(out _), "changed carrying program");
                    data.Update(); data.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = retreat; data.ApplyModifiedPropertiesWithoutUndo();
                    Require(s.TryResume(out e), e); await CheckCompletion(r, "restore program and resume holding");
                }
                finally { s.Pause(); SetWaypoints(s, originalData); UnityEngine.Object.DestroyImmediate(copy); }
            }
            OpenMenus(); await Task.Delay(150, token);
            foreach (var root in Roots()) Require(root.Q<Label>("gripper-cycle-state").text.Contains("완료") && !root.Q<Button>("gripper-cycle-stop").enabledSelf,
                "Completed auto-cycle UI incorrect.");
            Require(hashes.All(p => Hash(p.Key) == p.Value), "Source code/assets changed.");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"), robots = 2, completedCycles = cycles,
                rejectedRequestsPreserved = rejected, emptyPauses, heldPauses, recoveredFaults = faults, manualOverrides,
                maxPlacedGripPointPositionErrorMetres = maxPlacedPosition, maxPlacedGripPointRotationErrorDegrees = maxPlacedRotation,
                exactlyOneAutomaticGripAndReleasePerCompletedCycle = true, noImplicitRegripAfterLoss = true,
                codeAndAssetsUnchanged = true, hashes, uiStateRead = true, directButtonClicks = 0,
                scope = "LW-08 process 4: per-robot four-waypoint pick/transport/release-in-place cycles; 10x simulation, no two-robot handoff, collision/force/gravity or Cartesian FANUC trajectory claim", rows };
            Write("pick-place-play-verification.json", report);
            return new { report.passed, report.robots, report.completedCycles, report.rejectedRequestsPreserved, report.emptyPauses, report.heldPauses,
                report.recoveredFaults, report.manualOverrides, report.maxPlacedGripPointPositionErrorMetres, report.maxPlacedGripPointRotationErrorDegrees,
                report.exactlyOneAutomaticGripAndReleasePerCompletedCycle, report.codeAndAssetsUnchanged };
        }
        finally
        {
            EditorApplication.isPaused = false; Time.timeScale = originalScale;
            foreach (var lab in labs)
            {
                lab.GetComponent<RobotWaypointSequence>().Pause(); lab.Gripper.Stop();
                if (lab.Gripper.HeldItem != null) lab.Gripper.TryRelease(out _);
            }
        }
    }

    private static void OpenMenus()
    { foreach (string menu in new[] { "LineWatch/SCARA 4축 조작 테스트", "LineWatch/FANUC 조작 테스트" }) Require(EditorApplication.ExecuteMenuItem(menu), "Menu missing."); }
    private static VisualElement[] Roots() => new[] { EditorWindow.GetWindow<ScaraJointLabWindow>().rootVisualElement, EditorWindow.GetWindow<FanucJointLabWindow>().rootVisualElement };
    private static void SetWaypoints(RobotWaypointSequence sequence, RobotWaypointSet data)
    { var s = new SerializedObject(sequence); s.FindProperty("waypoints").objectReferenceValue = data; s.ApplyModifiedPropertiesWithoutUndo(); }
    private static async Task Until(Func<bool> condition, CancellationToken token)
    { var watch = Stopwatch.StartNew(); while (!condition()) { token.ThrowIfCancellationRequested(); Require(watch.Elapsed.TotalSeconds < 15, "Wait timed out."); await Task.Delay(1, token); } }
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW08"); File.WriteAllText("Logs/LW08/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
