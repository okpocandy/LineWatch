using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using State = LineWatch.RobotWaypointSequence.RunState;

public static class VerifyPathFaultRecovery
{
    public static string OpenScene(bool robotLab)
    {
        Require(!robotLab, "RobotLab was retired. Use LineWatch; historical RobotLab verification requires restoring its archived scene.");
        Require(!EditorApplication.isPlaying && !UnityEngine.SceneManagement.SceneManager.GetActiveScene().isDirty, "Stop Play and preserve any unsaved user scene edits before switching.");
        var scene = UnityEditor.SceneManagement.EditorSceneManager.OpenScene("Assets/01.Scenes/LineWatch.unity");
        return scene.path;
    }

    public static async Task<string> UiLockTiming()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>();
        var s = EditorWindow.GetWindow<ScaraJointLabWindow>(); s.Show(); s.RefreshLab();
        var f = EditorWindow.GetWindow<FanucJointLabWindow>(); f.Show(); f.RefreshLab();
        var rows = new List<object>();
        try
        {
            Require(c.TryExecute(CellRunCommand.Start, out var e), e); var start = DateTime.UtcNow;
            for (int i = 0; i < 20; i++)
            {
                bool scara = s.rootVisualElement.Q<Button>("scara-ik-move").enabledInHierarchy;
                bool fanuc = f.rootVisualElement.Q<Button>("fanuc-ik-move").enabledInHierarchy;
                Require(!c.CanIssueManualCommand(l.Scara, out _) && !c.CanIssueManualCommand(l.Fanuc, out _), "Runtime gate allowed a manual command while Running.");
                rows.Add(new { milliseconds = (DateTime.UtcNow - start).TotalMilliseconds, scara, fanuc });
                if (!scara && !fanuc) break;
                await Task.Delay(100);
            }
            Require(!s.rootVisualElement.Q<Button>("scara-ik-move").enabledInHierarchy && !f.rootVisualElement.Q<Button>("fanuc-ik-move").enabledInHierarchy,
                "IK controls failed to reflect the runtime lock within two seconds.");
            var report = new { passed = true, runtimeGateAlwaysLocked = true, directButtonClicks = 0, callbacksInjected = 0, rows };
            Write("path-recovery-ui-lock-timing.json", report); return JsonConvert.SerializeObject(report);
        }
        finally { c.TryExecute(CellRunCommand.Stop, out _); }
    }

    public static async Task<string> Reproduce()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var q = l.Scara.GetComponent<RobotWaypointSequence>();
        var source = q.Waypoints; var copy = UnityEngine.Object.Instantiate(source); var scale = Time.timeScale;
        try
        {
            SetWaypoints(q, copy); Time.timeScale = .1f;
            await Until(() => l.GetComponent<CellProductTracker>().Count == 1);
            Require(q.TryStart(out var e), e); q.Pause(); Require(c.TryTakeControl(q, out e), e);
            Require(c.TryExecute(CellRunCommand.Start, out e), e);
            SetRetreat(copy, source.Retreat.PositionMetres + Vector3.right * .01f);
            await Until(() => q.State == State.Faulted && c.IsFaulted);
            SetRetreat(copy, source.Retreat.PositionMetres); await Ticks(2);
            bool eligible = c.CanExecute(CellRunCommand.Reset, out e);
            var report = new { reproduced = !eligible && q.State == State.Faulted && c.IsFaulted, error = e,
                step = q.StepIndex, owned = c.ControlledSequence == q, physicalCount = l.GetComponent<CellProductTracker>().Count };
            Write("path-fault-recovery-before.json", report); return JsonConvert.SerializeObject(report);
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetWaypoints(q, source); UnityEngine.Object.Destroy(copy); }
    }

    public static async Task<string> Restored(int scenario)
    {
        Require(scenario >= 0 && scenario <= 4, "Unknown restored scenario.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = (scenario == 1 || scenario == 3 ? l.Fanuc.gameObject : l.Scara.gameObject).GetComponent<RobotGripper>();
        var q = g.GetComponent<RobotWaypointSequence>(); var source = q.Waypoints; var copy = UnityEngine.Object.Instantiate(source);
        var hashes = Sources(); float scale = Time.timeScale; int rejected = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
            Require(t.TryGetItem("P-000001", out var item), "Initial product missing.");
            if (scenario == 3 || scenario == 4)
            {
                var sg = l.Scara.GetComponent<RobotGripper>();
                await Move(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
                await Move(sg, item, l.HandoffPoint); Require(sg.TryRelease(out e), e);
                var retreat = l.ScaraHandoffWaypoints.Retreat;
                Require(l.Scara.TryMoveTcp(retreat.PositionMetres, Yaw(retreat.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out e), e);
                await Until(() => sg.IsStopped);
                if (scenario == 4)
                {
                    var fg = l.Fanuc.GetComponent<RobotGripper>();
                    await Move(fg, item, l.HandoffPoint); Require(fg.TryGrip(item, out e), e);
                    await Move(fg, item, l.OutputA); Require(fg.TryRelease(out e), e);
                    Require(t.TryRemove("WAIT-B", ProductLocation.Waiting, out e), e);
                    UnityEngine.Object.Destroy(item.gameObject); await Ticks(2);
                    Require(t.TryGetItem("WAIT-A", out item), "Waiting item missing.");
                    item.Body.position = l.InputStop.position; item.Body.rotation = l.InputStop.rotation;
                    Require(t.TryFeedNext(out var fed, out e) && fed.Id == "WAIT-A", e);
                    AddWaiting(l, "TAIL-A", .6f); AddWaiting(l, "TAIL-B", .9f);
                }
            }
            if (scenario >= 2)
            { await Move(g, item, scenario == 3 ? l.HandoffPoint : l.InputStop); Require(g.TryGrip(item, out var e), e); }
            SetWaypoints(q, copy); Require(q.TryStart(out var startError), startError); q.Pause();
            Require(c.TryTakeControl(q, out startError), startError);
            string records = Records(l), previousArchive = q.LastAcknowledgedFaultReason;
            for (int cycle = 0; cycle < 2; cycle++)
            {
                Time.timeScale = cycle == 0 ? .1f : 5f;
                Require(c.TryExecute(CellRunCommand.Start, out var e), e);
                if (cycle == 1) await Until(() => q.StepIndex >= 2 || c.IsFaulted);
                Require(q.IsBusy && !q.IsPickPlace, "Movement-only fixture did not reach the saved stage.");
                SetRetreat(copy, source.Retreat.PositionMetres + Vector3.right * .01f);
                await Until(() => q.State == State.Faulted && c.IsFaulted);
                Require(q.LastAcknowledgedFaultReason == previousArchive, "A new fault erased the previous path acknowledgement.");
                if (cycle == 0 && scenario >= 2) { t.enabled = false; t.enabled = true; }
                string full = Full(l);
                Require(!c.TryExecute(CellRunCommand.Reset, out e) && !string.IsNullOrEmpty(e) && Full(l) == full,
                    "Changed program acknowledgement partially cleared faults/data."); rejected++;
                SetRetreat(copy, source.Retreat.PositionMetres);
                q.enabled = false; q.enabled = true; c.enabled = false; c.enabled = true;
                Time.timeScale = 1; await Ticks(3);
                Require(q.State == State.Faulted && c.IsFaulted && q.LastAcknowledgedFaultReason == previousArchive,
                    "Restoring components/program cleared a fault automatically.");
                full = Full(l);
                for (int i = 0; i < 100; i++) Require(c.CanExecute(CellRunCommand.Reset, out e), e);
                Require(Full(l) == full, "Eligibility queries changed faults, commands or records.");
                var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show(); w.RefreshCell(); await Task.Delay(150);
                string label = g.Scara != null ? "cell-run-scara" : "cell-run-fanuc";
                Require(w.rootVisualElement.Q<Button>("cell-run-reset").enabledInHierarchy && w.rootVisualElement.Q<Label>(label).text.Contains("경로 고장 유지"),
                    "Active path fault missing from common UI.");
                Acknowledge(l, q); previousArchive = q.LastAcknowledgedFaultReason;
                string data = Data(l); await Ticks(3); await Task.Delay(150);
                Require(q.State == State.Paused && c.State == CellRunState.Stopped && Data(l) == data && Records(l) == records,
                    "Acknowledgement auto-started or changed data/saved progress.");
                Require(w.rootVisualElement.Q<Label>(label).text.Contains("최근 경로 명시 해제") && !w.rootVisualElement.Q<Button>("cell-run-reset").enabledInHierarchy,
                    "Path acknowledgement history or duplicate rejection missing from UI.");
                full = Full(l);
                Require(!c.TryExecute(CellRunCommand.Reset, out _) && !q.TryResume(out _) && !q.TryStart(out _) && Full(l) == full,
                    "Duplicate reset/individual resume/start bypassed ownership."); rejected += 3;
                if (scenario >= 2) Require(g.HeldItem == item && item.Owner == g, "Acknowledgement dropped the tracked holding.");
            }
            Time.timeScale = 5; Require(c.TryExecute(CellRunCommand.Start, out var finalError), finalError);
            await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && Records(l) == records && q.GripCount == 0 && q.ReleaseCount == 0,
                q.LastError ?? c.FaultReason ?? "Separate common start failed or repeated a product action.");
            Require(c.TryExecute(CellRunCommand.Stop, out finalError) && c.TryReleaseControl(out finalError), finalError);
            if (scenario == 4)
            {
                var candidate = UnityEngine.Object.Instantiate(l.ProductPrefab); candidate.Body.position = l.InputStop.position + Vector3.right * 1.3f;
                Require(!t.TryRegister(candidate, "P-000001", ProductDestination.A, out _) && !t.TryGetItem("P-000001", out _), "Terminal ID/instance history was erased.");
            }
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, pathAcknowledgements = 2,
                eligibilityQueriesPreserved = 200, rejectedRequestsPreserved = rejected, completedMovementPrograms = 1,
                registeredProducts = t.Count, completedProducts = scenario == 4 ? 1 : 0, removedProducts = scenario == 4 ? 1 : 0,
                physicallyConfirmedResidual = 3, unconfirmedProducts = 0, waitingOrder = t.WaitingSnapshot(),
                motionCommandsAtAcknowledgement = 0, sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0,
                scope = "Only adopted movement programs: step 0 and later step recovery, same records/physical owners, no dispatch until separate Start.", hashes };
            Write("path-fault-recovery-restored-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.pathAcknowledgements, report.rejectedRequestsPreserved, report.registeredProducts, report.completedMovementPrograms });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetWaypoints(q, source); UnityEngine.Object.Destroy(copy); }
    }

    public static async Task<string> Blocked(int scenario)
    {
        Require(scenario >= 0 && scenario <= 5, "Unknown blocked scenario.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = l.Scara.GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>();
        var source = q.Waypoints; var copy = UnityEngine.Object.Instantiate(source); var hashes = Sources(); float scale = Time.timeScale, link = 0;
        bool repaired = false; float originalMaximum = l.Scara.Axis3.MaximumMetres;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
            Require(t.TryGetItem("P-000001", out var item), "Initial product missing."); SetWaypoints(q, copy);
            if (scenario == 1) { await Move(g, item, l.InputStop); Require(g.TryGrip(item, out var e), e); }
            if (scenario == 3)
            {
                var lab = g.GetComponent<RobotGripperLab>(); Require(lab != null && lab.TryPrepare(out _), "Prepare pick/place trial.");
                Require(q.TryStartPickPlace(g, lab.TrialItem, out var e), e);
            }
            else Require(q.TryStart(out var e), e);
            q.Pause(); if (scenario != 2 && scenario != 3) Require(c.TryTakeControl(q, out var e2), e2);
            Time.timeScale = .1f;
            if (scenario == 2 || scenario == 3) Require(q.TryResume(out var e), e);
            else Require(c.TryExecute(CellRunCommand.Start, out var e), e);
            SetRetreat(copy, source.Retreat.PositionMetres + Vector3.right * .01f);
            await Until(() => q.State == State.Faulted && c.IsFaulted);
            SetRetreat(copy, source.Retreat.PositionMetres); t.enabled = false; t.enabled = true; Time.timeScale = 1;
            var stationPose = item.Body.position;
            if (scenario == 0)
            { var so = new SerializedObject(l.Scara); var f = so.FindProperty("fkLink1Metres"); link = f.floatValue; f.floatValue = link + .01f; so.ApplyModifiedPropertiesWithoutUndo(); }
            else if (scenario == 1) { item.gameObject.SetActive(false); item.gameObject.SetActive(true); }
            else if (scenario == 4) item.Body.position = stationPose + Vector3.right * .01f;
            else if (scenario == 5)
            {
                var so = new SerializedObject(l.Scara.Axis3);
                so.FindProperty("maximumMetres").floatValue = Mathf.Max(l.Scara.Axis3.CurrentPosition, .001f) + .001f;
                so.ApplyModifiedPropertiesWithoutUndo();
                Require(c.CanExecute(CellRunCommand.Reset, out _), "Fixture must reach final path validation beyond the cheap eligibility query.");
            }
            await Ticks(3); string before = Full(l);
            Require(!c.TryExecute(CellRunCommand.Reset, out var error) && !string.IsNullOrEmpty(error) && Full(l) == before && q.State == State.Faulted && c.IsFaulted && t.IsFaulted,
                "Failed final validation partially acknowledged a path/tracker/cell or changed data.");
            if (scenario == 0 || scenario == 4 || scenario == 5)
            {
                if (scenario == 0) { var so = new SerializedObject(l.Scara); so.FindProperty("fkLink1Metres").floatValue = link; so.ApplyModifiedPropertiesWithoutUndo(); }
                else if (scenario == 4) item.Body.position = stationPose;
                else { var so = new SerializedObject(l.Scara.Axis3); so.FindProperty("maximumMetres").floatValue = originalMaximum; so.ApplyModifiedPropertiesWithoutUndo(); }
                await Ticks(2); Require(q.State == State.Faulted && c.IsFaulted && t.IsFaulted, "Repair auto-cleared a fault.");
                Acknowledge(l, q); repaired = true;
            }
            else Require(!c.CanExecute(CellRunCommand.Reset, out _) && q.State == State.Faulted, "Unsupported/lost-owner path became recoverable.");
            Require(t.Count == 3 && t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B" }), "Negative recovery changed totals/FIFO.");
            if (scenario == 1) Require(item.Owner == null && g.HeldItem == null && t.ScaraProductId == "P-000001", "Owner loss normalized into normal holding.");
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, rejectedRequestsPreserved = 1,
                noPartialLatchClearOnFailure = true, repairedThenExplicitlyAcknowledged = repaired, unconfirmedProducts = scenario == 1 ? 1 : 0,
                finalPathPreflightRejected = scenario == 5,
                automaticRecovery = false, motionCommandsAtAcknowledgement = 0, sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0,
                scope = "FK/station mismatch, lost Owner, unadopted movement/pick-place path faults preserve all three latches on failure.", hashes };
            Write("path-fault-recovery-blocked-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.noPartialLatchClearOnFailure, report.repairedThenExplicitlyAcknowledged, report.unconfirmedProducts });
        }
        finally
        {
            if (link != 0) { var so = new SerializedObject(l.Scara); so.FindProperty("fkLink1Metres").floatValue = link; so.ApplyModifiedPropertiesWithoutUndo(); }
            if (scenario == 5) { var so = new SerializedObject(l.Scara.Axis3); so.FindProperty("maximumMetres").floatValue = originalMaximum; so.ApplyModifiedPropertiesWithoutUndo(); }
            Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetWaypoints(q, source); UnityEngine.Object.Destroy(copy);
        }
    }

    private static void Acknowledge(LineWatchCellLayout l, RobotWaypointSequence q)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        string data = Data(l), cellCause = c.FaultReason, pathCause = q.LastError, trackerCause = t.FaultReason, archive = t.LastAcknowledgedFaultReason;
        bool trackingFault = t.IsFaulted;
        Require(c.TryExecute(CellRunCommand.Reset, out var error), error);
        Require(c.State == CellRunState.Stopped && q.State == State.Paused && !c.IsFaulted && !t.IsFaulted &&
            c.FaultReason == null && q.LastError == null && t.FaultReason == null && c.LastAcknowledgedFaultReason == cellCause &&
            q.LastAcknowledgedFaultReason == pathCause && t.LastAcknowledgedFaultReason == (trackingFault ? trackerCause : archive) && Data(l) == data,
            "Acknowledgement changed saved data/motor commands or lost independent causes.");
    }

    private static GrippableItem AddWaiting(LineWatchCellLayout l, string id, float offset)
    {
        var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset;
        Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e); return item;
    }
    private static async Task Move(RobotGripper g, GrippableItem item, Transform station)
    {
        var point = item.GetGripPoint(g.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc);
        var rotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = station.TransformPoint(point.localPosition) - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        var local = Quaternion.Inverse(g.RobotFrame.rotation) * rotation; string error;
        bool ok = g.Scara != null ? g.Scara.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), Yaw(local), ScaraJointLab.IkSelection.PositiveElbow, out _, out error)
            : g.Fanuc.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), local, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error);
        Require(ok, error); await Until(() => g.IsStopped); await Ticks(2);
    }
    private static string Records(LineWatchCellLayout l)
    {
        var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(),
            slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            physical = t.Snapshot().Select(r => new { r.Id, instance = t.TryGetItem(r.Id, out var item) ? item.GetInstanceID() : 0,
                owner = item != null && item.Owner != null ? item.Owner.GetInstanceID() : 0 }) });
    }
    // Excludes only fault/state fields that the acknowledgement intentionally changes.
    private static string Data(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); return JsonConvert.SerializeObject(new { records = Records(l), c.HasControlledSequence,
            controlled = c.ControlledSequence != null ? c.ControlledSequence.GetInstanceID() : 0,
            robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() }.Select(g => {
                var q = g.GetComponent<RobotWaypointSequence>(); var held = g.HeldItem;
                return new { version = g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion,
                    current = g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition } : g.Fanuc.Joints.Select(a => a.CurrentPosition).ToArray(),
                    targets = g.Scara != null ? new[] { g.Scara.Axis1.TargetPosition, g.Scara.Axis2.TargetPosition, g.Scara.Axis3.TargetPosition, g.Scara.Axis4.TargetPosition } : g.Fanuc.Joints.Select(a => a.TargetPosition).ToArray(),
                    q.StepIndex, q.GripCount, q.ReleaseCount, q.IsPickPlace, q.LastArrivedStepIndex,
                    arrivalPosition = V(q.LastArrivalPose.position), arrivalRotation = Q(q.LastArrivalPose.rotation),
                    held = held != null ? held.GetInstanceID() : 0, position = held != null ? V(held.Body.position) : null, rotation = held != null ? Q(held.Body.rotation) : null };
            }) });
    }
    private static string Full(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.LastAcknowledgedFaultReason, trackerFault = t.IsFaulted,
            trackerCause = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason,
            paths = new[] { l.Scara.GetComponent<RobotWaypointSequence>(), l.Fanuc.GetComponent<RobotWaypointSequence>() }
                .Select(q => new { q.State, q.LastError, q.LastAcknowledgedFaultReason }), data = Data(l) });
    }
    private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }
        .SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))).Replace("-", ""); }
    private static void CheckSources(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification changed code/scene/assets.");

    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use a fresh unpaused LineWatch Play session.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch."); return l;
    }
    private static void SetWaypoints(RobotWaypointSequence q, RobotWaypointSet set)
    { var so = new SerializedObject(q); so.FindProperty("waypoints").objectReferenceValue = set; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetRetreat(RobotWaypointSet set, Vector3 position)
    { var so = new SerializedObject(set); so.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = position; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static async Task Ticks(int n) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= n * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
