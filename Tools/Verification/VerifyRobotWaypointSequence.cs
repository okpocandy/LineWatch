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

public static class VerifyRobotWaypointSequence
{
    private static readonly string[] Prefabs = { "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab" };
    private static readonly string[] Assets = { "Assets/04.ScriptableObject/ScaraLabWaypoints.asset", "Assets/04.ScriptableObject/FanucLabWaypoints.asset" };
    private static readonly string[] Protected = { "Assets/01.Scenes/RobotLab.unity", "ProjectSettings/ProjectSettings.asset" };

    public static object Wire()
    {
        Require(!EditorApplication.isPlaying, "Wire in Edit Mode.");
        var before = Protected.ToDictionary(p => p, Hash);
        for (int i = 0; i < 2; i++)
        {
            var root = PrefabUtility.LoadPrefabContents(Prefabs[i]);
            try
            {
                var sequence = root.GetComponent<RobotWaypointSequence>() ?? root.AddComponent<RobotWaypointSequence>();
                var s = new SerializedObject(sequence);
                s.FindProperty("waypoints").objectReferenceValue = AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(Assets[i]);
                s.FindProperty("scara").objectReferenceValue = root.GetComponent<ScaraJointLab>();
                s.FindProperty("fanuc").objectReferenceValue = root.GetComponent<FanucJointLab>();
                s.ApplyModifiedPropertiesWithoutUndo(); PrefabUtility.SaveAsPrefabAsset(root, Prefabs[i]);
            }
            finally { PrefabUtility.UnloadPrefabContents(root); }
        }
        Require(before.All(p => Hash(p.Key) == p.Value), "User scene/settings changed.");
        return Edit();
    }

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        var sequences = UnityEngine.Object.FindObjectsByType<RobotWaypointSequence>(FindObjectsSortMode.None);
        Require(sequences.Length == 2 && sequences.All(s => s.IsReady && s.State == State.Idle), "Expected two idle wired sequences.");
        foreach (var sequence in sequences)
        {
            Require(!sequence.TryStart(out var error) && !string.IsNullOrEmpty(error) && !sequence.TryResume(out _), "Edit Mode command accepted.");
        }
        foreach (string menu in new[] { "LineWatch/SCARA 4축 조작 테스트", "LineWatch/FANUC 조작 테스트" }) Require(EditorApplication.ExecuteMenuItem(menu), "Menu missing.");
        var roots = new[] { EditorWindow.GetWindow<ScaraJointLabWindow>().rootVisualElement, EditorWindow.GetWindow<FanucJointLabWindow>().rootVisualElement };
        foreach (var root in roots) Require(!root.Q<Button>("waypoint-start").enabledSelf && !root.Q<Button>("waypoint-resume").enabledSelf, "Edit UI not locked.");
        Require(roots[0].Q<UnityEditor.UIElements.ObjectField>("waypoint-data").value == sequences.Single(s => s.Waypoints.Robot == RobotWaypointSet.RobotType.Scara).Waypoints &&
            roots[1].Q<UnityEditor.UIElements.ObjectField>("waypoint-data").value == sequences.Single(s => s.Waypoints.Robot == RobotWaypointSet.RobotType.Fanuc).Waypoints, "Displayed waypoint binding is incorrect.");
        var report = new { passed = true, sequences = 2, editCommandsRejected = true, uiReadOnly = true, directButtonClicks = 0 };
        Directory.CreateDirectory("Logs/LW08"); File.WriteAllText("Logs/LW08/sequence-edit-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;
    }

    public static async Task<object> Play()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use unpaused Play Mode.");
        var scara = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        var fanuc = UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        var sequences = new[] { scara.GetComponent<RobotWaypointSequence>(), fanuc.GetComponent<RobotWaypointSequence>() };
        Require(sequences.All(s => s != null && s.IsReady && s.State == State.Idle), "Sequences must start idle.");
        var hashes = Protected.Concat(Prefabs).Concat(Assets).ToDictionary(p => p, Hash);
        var token = scara.destroyCancellationToken;
        float originalScale = Time.timeScale;
        var rows = new List<object>(); var endpoints = new List<object>();
        int completeCycles = 0, rejected = 0, stopChecks = 0, faultChecks = 0, intermediateArrivals = 0;
        float maxPosition = 0, maxRotation = 0;
        var allOriginalData = sequences.Select(s => s.Waypoints).ToArray();
        bool disabledAxis = fanuc.Joints[5].enabled;
        float[] Current(int robot) => robot == 0 ? new[] { scara.Axis1.CurrentPosition, scara.Axis2.CurrentPosition, scara.Axis3.CurrentPosition, scara.Axis4.CurrentPosition }
            : fanuc.Joints.Select(j => j.CurrentPosition).ToArray();
        bool AtRest(int robot) => robot == 0 ? !scara.IsTrajectoryMoving && !scara.Axis1.IsMoving && !scara.Axis2.IsMoving && !scara.Axis3.IsMoving && !scara.Axis4.IsMoving
            : !fanuc.IsTcpMoving && fanuc.Joints.All(j => !j.IsMoving);
        string Commands(int robot) => JsonConvert.SerializeObject(new { current = Current(robot), version = robot == 0 ? scara.CommandVersion : fanuc.CommandVersion,
            targets = robot == 0 ? new[] { scara.Axis1.TargetPosition, scara.Axis2.TargetPosition, scara.Axis3.TargetPosition, scara.Axis4.TargetPosition } : fanuc.Joints.Select(j => j.TargetPosition).ToArray() });
        async Task Hold(int robot)
        {
            var q = Current(robot); double t = Time.fixedTimeAsDouble;
            await Until(() => Time.fixedTimeAsDouble - t >= .1, token);
            Require(q.SequenceEqual(Current(robot)) && AtRest(robot), "Stopped group drifted."); stopChecks++;
        }
        void Reject(int robot, Func<bool> request)
        {
            var s = sequences[robot]; string before = Commands(robot); var state = s.State; int step = s.StepIndex;
            Require(!request() && before == Commands(robot) && state == s.State && step == s.StepIndex, "Rejected request changed commands/progress."); rejected++;
        }
        async Task Home(int robot)
        {
            sequences[robot].Pause();
            if (robot == 0) scara.HomeAll(); else fanuc.HomeAll();
            await Until(() => AtRest(robot), token);
        }
        async Task Cycle(int robot, bool interrupt)
        {
            var s = sequences[robot]; var data = s.Waypoints; var expected = new[] { data.Approach, data.Work, data.Approach, data.Retreat };
            var old = Current(robot); Require(s.TryStart(out var error), "Start rejected: " + error);
            Require(old.SequenceEqual(Current(robot)), "Start teleported a joint.");
            int previous = 0; bool stopped = false;
            var watch = Stopwatch.StartNew();
            while (s.State != State.Completed)
            {
                Require(watch.Elapsed.TotalSeconds < 20 && s.State != State.Faulted, "Cycle aborted: " + s.LastError);
                if (interrupt && !stopped && s.StepIndex == 1)
                {
                    var q = Current(robot); await Until(() => !q.SequenceEqual(Current(robot)), token);
                    int held = s.StepIndex; s.Pause(); await Hold(robot);
                    Require(s.State == State.Paused && s.StepIndex == held, "Pause skipped a step.");
                    EditorApplication.isPaused = true; Reject(robot, () => s.TryResume(out _)); EditorApplication.isPaused = false;
                    Time.timeScale = 0; Reject(robot, () => s.TryResume(out _)); Time.timeScale = 10;
                    var start = Current(robot); Require(s.TryResume(out error) && start.SequenceEqual(Current(robot)), "Resume did not plan from actual state: " + error);
                    stopped = true;
                }
                if (s.StepIndex > previous)
                {
                    Require(s.StepIndex == previous + 1, "Sequence skipped an endpoint.");
                    Require(s.LastArrivedStepIndex == previous, "Missing arrival snapshot.");
                    var target = expected[previous]; var actual = s.LastArrivalPose;
                    float angle = Angle(actual.rotation, target.Rotation);
                    float distance = Vector3.Distance(actual.position, target.PositionMetres);
                    Require(distance <= .002f && angle <= .5f, "Intermediate actual arrival missed tolerance.");
                    maxPosition = Mathf.Max(maxPosition, distance); maxRotation = Mathf.Max(maxRotation, angle);
                    endpoints.Add(new { robot, step = previous, positionMetres = V(actual.position), rotation = Q(actual.rotation), distance, angle });
                    intermediateArrivals++; previous = s.StepIndex;
                }
                await Task.Delay(1, token);
            }
            // Completed can be observed after the while condition before the final transition sample.
            if (previous == 3)
            {
                Require(s.LastArrivedStepIndex == 3, "Missing final arrival snapshot.");
                var actual = s.LastArrivalPose;
                float a = Angle(actual.rotation, data.Retreat.Rotation);
                float d = Vector3.Distance(actual.position, data.Retreat.PositionMetres); Require(d <= .002f && a <= .5f, "Final arrival missed tolerance.");
                endpoints.Add(new { robot, step = 3, positionMetres = V(actual.position), rotation = Q(actual.rotation), distance = d, angle = a });
                maxPosition = Mathf.Max(maxPosition, d); maxRotation = Mathf.Max(maxRotation, a); intermediateArrivals++;
            }
            Require(s.StepIndex == 4 && AtRest(robot), "Completed without all four stages.");
            rows.Add(new { robot, interrupted = interrupt, passed = true }); completeCycles++;
        }
        try
        {
            Time.timeScale = 10;
            Require(sequences.All(s => s.State == State.Idle) && AtRest(0) && AtRest(1), "Play auto-started a sequence.");
            for (int robot = 0; robot < 2; robot++)
            {
                await Home(robot); await Cycle(robot, true); await Cycle(robot, false);
                var s = sequences[robot]; await Home(robot);
                var copy = UnityEngine.Object.Instantiate(s.Waypoints);
                try
                {
                    SetData(s, copy);
                    var fields = new SerializedObject(copy);
                    var work = copy.Work.PositionMetres; var approach = copy.Approach.PositionMetres;
                    SetPosition(fields, "retreat", new Vector3(3, approach.y, 0));
                    Reject(robot, () => s.TryStart(out _)); // Full-route failure before the first motor command.
                    SetPosition(fields, "retreat", allOriginalData[robot].Retreat.PositionMetres);
                    Require(s.TryStart(out var error), error);
                    Reject(robot, () => s.TryStart(out _));
                    await Until(() => s.StepIndex == 1, token);
                    SetPosition(fields, "work", work + Vector3.up * .001f);
                    await Until(() => s.State == State.Faulted, token); Require(s.StepIndex == 1 && AtRest(robot), "Changed data did not stop/retain progress.");
                    await Hold(robot); Reject(robot, () => s.TryResume(out _)); faultChecks++;
                    SetPosition(fields, "work", work);
                    Require(s.TryResume(out error), error); await Until(() => s.State == State.Completed, token);
                }
                finally { s.Pause(); SetData(s, allOriginalData[robot]); UnityEngine.Object.DestroyImmediate(copy); }
                await Home(robot); Require(s.TryStart(out var e), e); await Until(() => s.StepIndex == 1, token);
                uint version = robot == 0 ? scara.CommandVersion : fanuc.CommandVersion;
                if (robot == 0) Require(scara.MoveJoint(0, 5), "Manual override rejected."); else Require(fanuc.MoveJoint(0, 5), "Manual override rejected.");
                await Until(() => s.State == State.Paused, token);
                Require(s.StepIndex == 1 && (robot == 0 ? scara.CommandVersion : fanuc.CommandVersion) != version, "External command did not interrupt sequence.");
                Require((robot == 0 ? scara.Axis1.TargetPosition : fanuc.Joints[0].TargetPosition) == 5, "Sequence overwrote manual target.");
                await Until(() => AtRest(robot), token); await Hold(robot);
                s.enabled = false; s.enabled = true; Require(s.State == State.Paused, "Enable auto-resumed a sequence.");
                await Home(robot);
            }
            var fs = sequences[1]; Require(fs.TryStart(out var faultError), faultError);
            await Until(() => fs.StepIndex == 1, token); fanuc.Joints[5].enabled = false;
            await Until(() => fs.State == State.Faulted, token); Require(fs.StepIndex == 1 && AtRest(1), "Axis fault skipped a step.");
            await Hold(1); fanuc.Joints[5].enabled = disabledAxis;
            Require(fs.TryResume(out faultError), faultError); await Until(() => fs.State == State.Completed, token); faultChecks++;
            foreach (string menu in new[] { "LineWatch/SCARA 4축 조작 테스트", "LineWatch/FANUC 조작 테스트" }) Require(EditorApplication.ExecuteMenuItem(menu), "Menu missing.");
            await Task.Delay(150, token);
            foreach (var root in new[] { EditorWindow.GetWindow<ScaraJointLabWindow>().rootVisualElement, EditorWindow.GetWindow<FanucJointLabWindow>().rootVisualElement })
                Require(root.Q<Button>("waypoint-start").enabledSelf && !root.Q<Button>("waypoint-stop").enabledSelf && !string.IsNullOrEmpty(root.Q<Label>("waypoint-state").text), "Sequence UI state incorrect.");
            Require(hashes.All(p => Hash(p.Key) == p.Value), "Source assets changed.");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"), fullObservedCycles = completeCycles,
                observedActualEndpointArrivals = intermediateArrivals, rejectedRequestsPreserved = rejected, stopChecks, faultChecks,
                maxObservedActualPositionErrorMetres = maxPosition, maxObservedActualRotationErrorDegrees = maxRotation,
                sourceAssetsUnchanged = true, hashes, editModeVerifiedSeparately = true, uiStateRead = true, directButtonClicks = 0,
                scope = "LW-08 process 2: API-driven per-robot four-stage sequences, 10x simulation, latched actual endpoint arrivals, pause/resume and ownership/data/fault probes; no grasp, shared handoff, collision or acceleration verification", rows, endpoints };
            Directory.CreateDirectory("Logs/LW08"); File.WriteAllText("Logs/LW08/sequence-play-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;
        }
        finally
        {
            EditorApplication.isPaused = false; Time.timeScale = originalScale; fanuc.Joints[5].enabled = disabledAxis;
            foreach (var s in sequences) if (s != null) s.Pause();
            scara.StopAll(); fanuc.StopAll();
            for (int i = 0; i < 2; i++) SetData(sequences[i], allOriginalData[i]);
        }
    }

    public static async Task<object> Controls()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use unpaused Play Mode.");
        var scara = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        var fanuc = UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        var sequences = new[] { scara.GetComponent<RobotWaypointSequence>(), fanuc.GetComponent<RobotWaypointSequence>() };
        var token = scara.destroyCancellationToken; float originalScale = Time.timeScale;
        var codePaths = new[] { "Assets/02.Scripts/LineWatch/RobotWaypointSequence.cs", "Assets/02.Scripts/LineWatch/ScaraJointLab.cs", "Assets/02.Scripts/LineWatch/FanucJointLab.cs" };
        var hashes = codePaths.Concat(Protected).Concat(Prefabs).Concat(Assets).ToDictionary(p => p, Hash);
        int existingStops = 0, componentStops = 0;
        float[] Current(int r) => r == 0 ? new[] { scara.Axis1.CurrentPosition, scara.Axis2.CurrentPosition, scara.Axis3.CurrentPosition, scara.Axis4.CurrentPosition }
            : fanuc.Joints.Select(j => j.CurrentPosition).ToArray();
        bool AtRest(int r) => r == 0 ? !scara.IsTrajectoryMoving && !scara.Axis1.IsMoving && !scara.Axis2.IsMoving && !scara.Axis3.IsMoving && !scara.Axis4.IsMoving
            : !fanuc.IsTcpMoving && fanuc.Joints.All(j => !j.IsMoving);
        async Task Hold(int r)
        {
            var q = Current(r); int step = sequences[r].StepIndex; double t = Time.fixedTimeAsDouble;
            await Until(() => Time.fixedTimeAsDouble - t >= .1, token);
            Require(q.SequenceEqual(Current(r)) && AtRest(r) && step == sequences[r].StepIndex, "Stopped sequence auto-advanced.");
        }
        try
        {
            Time.timeScale = 10;
            for (int robot = 0; robot < 2; robot++)
            {
                var s = sequences[robot]; s.Pause();
                if (robot == 0) scara.HomeAll(); else fanuc.HomeAll();
                await Until(() => AtRest(robot), token);
                Require(s.TryStart(out var error), error); await Until(() => s.StepIndex == 1, token);
                if (robot == 0) scara.StopAll(); else fanuc.StopAll();
                await Until(() => s.State == State.Paused, token); await Hold(robot); existingStops++;
                Require(s.TryResume(out error), error); await Until(() => s.State == State.Completed, token);
                Require(s.TryStart(out error), error); await Until(() => s.StepIndex == 1, token);
                s.enabled = false;
                Require(s.State == State.Paused && AtRest(robot), "Disabling did not stop owned motion.");
                s.enabled = true; await Hold(robot); componentStops++;
                Require(s.State == State.Paused && s.TryResume(out error), error);
                await Until(() => s.State == State.Completed, token);
            }
            Require(hashes.All(p => Hash(p.Key) == p.Value), "Code/source assets changed.");
            var report = new { passed = true, existingStopApisPreservedProgress = existingStops, disabledSequenceStops = componentStops,
                resumedCyclesCompleted = 4, codeAndAssetsUnchanged = true, hashes, directButtonClicks = 0 };
            Directory.CreateDirectory("Logs/LW08"); File.WriteAllText("Logs/LW08/sequence-controls-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return new { report.passed, report.existingStopApisPreservedProgress, report.disabledSequenceStops, report.resumedCyclesCompleted, report.codeAndAssetsUnchanged };
        }
        finally { Time.timeScale = originalScale; foreach (var s in sequences) { s.enabled = true; s.Pause(); } scara.StopAll(); fanuc.StopAll(); }
    }

    private static void SetData(RobotWaypointSequence sequence, RobotWaypointSet data)
    { var s = new SerializedObject(sequence); s.FindProperty("waypoints").objectReferenceValue = data; s.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetPosition(SerializedObject s, string name, Vector3 value)
    { s.Update(); s.FindProperty(name).FindPropertyRelative("positionMetres").vector3Value = value; s.ApplyModifiedPropertiesWithoutUndo(); }
    private static async Task Until(Func<bool> condition, CancellationToken token)
    { var watch = Stopwatch.StartNew(); while (!condition()) { token.ThrowIfCancellationRequested(); Require(watch.Elapsed.TotalSeconds < 15, "Wait timed out."); await Task.Delay(1, token); } }
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static float Angle(Quaternion a, Quaternion b)
    { var d = Quaternion.Inverse(a) * b; return (float)(2 * Math.Atan2(Math.Sqrt((double)d.x * d.x + (double)d.y * d.y + (double)d.z * d.z), Math.Abs(d.w)) * 180 / Math.PI); }
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
