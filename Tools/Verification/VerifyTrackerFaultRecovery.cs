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

public static class VerifyTrackerFaultRecovery
{
    public static async Task<object> Restored(int scenario)
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var fg = l.Fanuc.GetComponent<RobotGripper>();
        var hashes = Sources(); float scale = Time.timeScale;
        try
        {
            Require(scenario >= 0 && scenario <= 3, "Unknown restored scenario.");
            Time.timeScale = 5; await Until(() => t.Count == 1 || t.IsFaulted);
            var waiting = AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
            Require(t.TryGetItem("P-000001", out var item), "Initial product missing.");
            var g = scenario == 2 ? fg : sg;
            if (scenario == 2 || scenario == 3)
            {
                await Move(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
                await Move(sg, item, l.HandoffPoint); Require(sg.TryRelease(out e), e);
                var retreat = l.ScaraHandoffWaypoints.Retreat;
                Require(l.Scara.TryMoveTcp(retreat.PositionMetres, Yaw(retreat.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out e), e);
                await Until(() => sg.IsStopped);
                if (scenario == 3)
                {
                    await Move(fg, item, l.HandoffPoint); Require(fg.TryGrip(item, out e), e);
                    await Move(fg, item, l.OutputA); Require(fg.TryRelease(out e), e);
                    Require(t.TryRemove("WAIT-B", ProductLocation.Waiting, out e), e);
                    UnityEngine.Object.Destroy(item.gameObject); await Ticks(2);
                    item = waiting; item.Body.position = l.InputStop.position; item.Body.rotation = l.InputStop.rotation;
                    Require(t.TryFeedNext(out var fed, out e) && fed.Id == "WAIT-A", e);
                    AddWaiting(l, "TAIL-A", .6f); AddWaiting(l, "TAIL-B", .9f);
                }
            }
            var q = g.GetComponent<RobotWaypointSequence>();
            if (scenario != 1)
            {
                await Move(g, item, scenario == 2 ? l.HandoffPoint : l.InputStop); Require(g.TryGrip(item, out var e), e);
                Require(q.TryStart(out e), e); q.Pause(); Require(c.TryTakeControl(q, out e), e);
                Time.timeScale = .1f; Require(c.TryExecute(CellRunCommand.Start, out e), e); await Ticks(1);
            }
            if (scenario == 0) { c.enabled = false; c.enabled = true; }
            if (scenario == 1)
            {
                var position = item.Body.position; item.Body.position = position + Vector3.right * .01f;
                await Until(() => t.IsFaulted && c.IsFaulted);
                Require(!c.CanExecute(CellRunCommand.Reset, out _), "Current station mismatch was recoverable.");
                item.Body.position = position;
            }
            else
            {
                t.enabled = false; Require(!c.CanExecute(CellRunCommand.Reset, out _), "Disabled tracker was recoverable."); t.enabled = true;
            }
            Time.timeScale = 1; await Ticks(3);
            Require(c.IsFaulted && t.IsFaulted, "Restoration cleared a latch automatically.");
            string cellCause = c.FaultReason, trackerCause = t.FaultReason, before = Full(l);
            for (int i = 0; i < 100; i++) Require(c.CanExecute(CellRunCommand.Reset, out var error), error);
            Require(Full(l) == before, "Recovery query changed latches, histories or data.");
            if (scenario == 0) Require(cellCause != trackerCause, "Fixture must preserve distinct first causes.");
            var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show(); w.RefreshCell(); await Task.Delay(200);
            Require(w.rootVisualElement.Q<Button>("cell-run-reset").enabledInHierarchy &&
                w.rootVisualElement.Q<Label>("cell-run-products").text.Contains(trackerCause), "Recoverable tracker fault missing from UI.");
            string records = Records(l); var stoppedPose = item.Body.position;
            Acknowledge(l);
            Require(c.State == CellRunState.Stopped && !c.IsFaulted && !t.IsFaulted, "Both latches were not acknowledged together.");
            await Ticks(3); await Task.Delay(150);
            Require(Records(l) == records && Vector3.Distance(item.Body.position, stoppedPose) < .00001f &&
                q.State == (scenario == 1 ? State.Idle : State.Paused), "Acknowledgement moved/advanced the robot or changed records.");
            Require(w.rootVisualElement.Q<Label>("cell-run-products").text.Contains("최근 추적 명시 해제") &&
                w.rootVisualElement.Q<HelpBox>("cell-run-fault").text.Contains(cellCause), "Independent acknowledgement causes missing from UI.");
            before = Full(l);
            Require(!c.TryExecute(CellRunCommand.Reset, out _) && Full(l) == before, "Duplicate acknowledgement changed data/history.");
            Time.timeScale = 5;
            Require(c.TryExecute(CellRunCommand.Start, out var e2), e2);
            if (scenario != 1)
            {
                await Until(() => q.State == State.Completed || c.IsFaulted);
                Require(q.State == State.Completed && g.HeldItem == item && item.Owner == g && q.GripCount == 0 && q.ReleaseCount == 0 && Records(l) == records,
                    q.LastError ?? c.FaultReason ?? "Explicit restart lost registered holding or records.");
            }
            else Require(q.State == State.Idle && Records(l) == records, "Permission-only Start created a program.");
            Require(c.TryExecute(CellRunCommand.Stop, out e2), e2);
            if (c.HasControlledSequence) Require(c.TryReleaseControl(out e2), e2);
            if (scenario == 3)
            {
                var candidate = UnityEngine.Object.Instantiate(l.ProductPrefab); candidate.Body.position = l.InputStop.position + Vector3.right * 1.3f;
                Require(!t.TryRegister(candidate, "P-000001", ProductDestination.A, out _) && !candidate.IsTracked && !t.TryGetItem("P-000001", out _),
                    "Tracker recovery erased terminal ID history or recreated the completed object.");
            }
            // A later cell-only acknowledgement must keep the previous tracker archive.
            c.enabled = false; c.enabled = true; await Ticks(2);
            Require(c.IsFaulted && !t.IsFaulted && t.LastAcknowledgedFaultReason == trackerCause, "Cell-only fault changed tracker acknowledgement history.");
            Acknowledge(l);
            Require(t.LastAcknowledgedFaultReason == trackerCause && c.State == CellRunState.Stopped, "Cell-only acknowledgement erased tracker history.");
            var snapshot = t.Snapshot(); int completed = snapshot.Count(r => r.Location == ProductLocation.OutputA || r.Location == ProductLocation.OutputB);
            int removed = snapshot.Count(r => r.Location == ProductLocation.Removed);
            Require(!t.IsFaulted && !c.IsFaulted && t.Count == (scenario == 3 ? 5 : 3) && completed == (scenario == 3 ? 1 : 0) && removed == (scenario == 3 ? 1 : 0), "Accounting totals changed.");
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, trackerAcknowledgements = 1, laterCellAcknowledgements = 1,
                eligibilityQueriesPreserved = 100, registeredProducts = t.Count, completedProducts = completed, removedProducts = removed,
                physicallyConfirmedResidual = t.Count - completed - removed, unconfirmedProducts = 0, cellCause, trackerCause,
                waitingOrder = t.WaitingSnapshot(), motionCommandsAtAcknowledgement = 0, sourcesPreserved = hashes.Count,
                directButtonClicks = 0, callbacksInjected = 0, hashes,
                scope = "Explicit tracker recovery with the same physical instances/owners/records. No identity rebinding, path-fault recovery or automatic restart." };
            Write("tracker-fault-recovery-restored-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.registeredProducts, report.completedProducts, report.removedProducts,
                report.physicallyConfirmedResidual, report.unconfirmedProducts, report.motionCommandsAtAcknowledgement });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
    }

    public static async Task<object> Blocked(int scenario)
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var q = sg.GetComponent<RobotWaypointSequence>();
        var original = q.Waypoints; RobotWaypointSet copy = null; var hashes = Sources(); float scale = Time.timeScale;
        float link = 0; bool repaired = false; int rejected = 0;
        void Reject(string label)
        {
            string before = Full(l);
            Require(!c.TryExecute(CellRunCommand.Reset, out var e) && !string.IsNullOrWhiteSpace(e) && Full(l) == before && c.IsFaulted && t.IsFaulted,
                "Recovery partially cleared latches/data on rejection: " + label); rejected++;
        }
        try
        {
            Require(scenario >= 0 && scenario <= 4, "Unknown blocked scenario.");
            Time.timeScale = 5; await Until(() => t.Count == 1 || t.IsFaulted); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
            Require(t.TryGetItem("P-000001", out var item), "Initial product missing.");
            if (scenario != 1)
            {
                if (scenario == 2 || scenario == 4) { copy = UnityEngine.Object.Instantiate(original); SetWaypoints(q, copy); }
                await Move(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
                Require(q.TryStart(out e), e); q.Pause(); if (scenario != 4) Require(c.TryTakeControl(q, out e), e);
            }
            if (scenario == 4)
            {
                // Unadopted faults still require a separate recovery policy.
                Time.timeScale = .1f; Require(q.TryResume(out var e), e);
                SetRetreat(copy, original.Retreat.PositionMetres + Vector3.right * .01f);
                await Until(() => q.State == State.Faulted && c.IsFaulted);
                SetRetreat(copy, original.Retreat.PositionMetres);
            }
            t.enabled = false; t.enabled = true; Time.timeScale = 1;
            Require(c.IsFaulted && t.IsFaulted, "Both latches required before negative recovery checks.");
            string cellCause = c.FaultReason, trackerCause = t.FaultReason;
            if (scenario == 0) { item.gameObject.SetActive(false); item.gameObject.SetActive(true); }
            else if (scenario == 1)
            {
                UnityEngine.Object.Destroy(item.gameObject); await Ticks(2);
                var replacement = UnityEngine.Object.Instantiate(l.ProductPrefab);
                replacement.Body.position = l.InputStop.position; replacement.Body.rotation = l.InputStop.rotation;
                Require(!replacement.IsTracked && !t.TryRegister(replacement, "P-000001", ProductDestination.A, out _), "Replacement rebound the missing original ID during a fault.");
                Require(!t.TryGetItem("P-000001", out _), "Replacement was mistaken for the registered physical instance.");
            }
            else if (scenario == 2) SetRetreat(copy, original.Retreat.PositionMetres + Vector3.right * .01f);
            else if (scenario == 3)
            {
                var so = new SerializedObject(l.Scara); var field = so.FindProperty("fkLink1Metres"); link = field.floatValue;
                field.floatValue = link + .01f; so.ApplyModifiedPropertiesWithoutUndo();
            }
            await Ticks(2);
            Require(c.FaultReason == cellCause && t.FaultReason == trackerCause && c.LastAcknowledgedFaultReason == null && t.LastAcknowledgedFaultReason == null,
                "Secondary evidence replaced first causes or created acknowledgement history.");
            Reject("scenario " + scenario);
            if (scenario == 2 || scenario == 3)
            {
                if (scenario == 2) SetRetreat(copy, original.Retreat.PositionMetres);
                else { var so = new SerializedObject(l.Scara); so.FindProperty("fkLink1Metres").floatValue = link; so.ApplyModifiedPropertiesWithoutUndo(); }
                await Ticks(2); Require(c.IsFaulted && t.IsFaulted, "Repair automatically cleared a latch.");
                Acknowledge(l); repaired = true;
                Require(q.State == State.Paused && sg.HeldItem == item && item.Owner == sg, "Post-repair recovery advanced/dropped the saved program.");
            }
            else
            {
                Require(!c.CanExecute(CellRunCommand.Reset, out _) && c.IsFaulted && t.IsFaulted && c.FaultReason == cellCause && t.FaultReason == trackerCause,
                    "Unresolved physical/path evidence became recoverable.");
                if (scenario == 0) Require(item.Owner == null && sg.HeldItem == null && t.ScaraProductId == "P-000001", "Lost owner was normalized into normal holding.");
                if (scenario == 4) Require(q.State == State.Faulted, "Path fault was silently changed to Paused.");
            }
            Require(t.Count == 3 && t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B" }), "Negative recovery changed totals/FIFO.");
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, rejectedRequestsPreserved = rejected,
                noPartialLatchClearOnFailure = true, repairedThenExplicitlyAcknowledged = repaired, automaticRecovery = false,
                unconfirmedProducts = scenario == 0 || scenario == 1 ? 1 : 0, registeredProducts = 3,
                sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, hashes,
                scope = "Unresolved holding loss/missing instance/path fault stay blocked; failed saved-path/FK validation preserves both latches before explicit post-repair recovery." };
            Write("tracker-fault-recovery-blocked-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.rejectedRequestsPreserved, report.noPartialLatchClearOnFailure,
                report.repairedThenExplicitlyAcknowledged, report.automaticRecovery, report.unconfirmedProducts });
        }
        finally
        {
            Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _);
            if (copy != null) { SetWaypoints(q, original); UnityEngine.Object.Destroy(copy); }
        }
    }

    private static void SetWaypoints(RobotWaypointSequence q, RobotWaypointSet set)
    { var so = new SerializedObject(q); so.FindProperty("waypoints").objectReferenceValue = set; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetRetreat(RobotWaypointSet set, Vector3 position)
    { var so = new SerializedObject(set); so.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = position; so.ApplyModifiedPropertiesWithoutUndo(); }

    private static void Acknowledge(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        string cellCause = c.FaultReason, trackerCause = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason, before = Data(l);
        bool trackingFault = t.IsFaulted;
        Require(c.TryExecute(CellRunCommand.Reset, out var e), e);
        Require(c.State == CellRunState.Stopped && !c.IsFaulted && !t.IsFaulted && c.FaultReason == null && t.FaultReason == null &&
            c.LastAcknowledgedFaultReason == cellCause && t.LastAcknowledgedFaultReason == (trackingFault ? trackerCause : trackerArchive) && Data(l) == before,
            "Acknowledgement partially cleared latches or changed data/path/motor commands.");
    }
    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use a fresh unpaused LineWatch Play session.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>(); Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch."); return l;
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
    private static string Data(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); return JsonConvert.SerializeObject(new { records = Records(l), c.HasControlledSequence,
            controlled = c.ControlledSequence != null ? c.ControlledSequence.GetInstanceID() : 0,
            robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() }.Select(g => {
                var q = g.GetComponent<RobotWaypointSequence>(); var held = g.HeldItem;
                return new { version = g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion,
                    current = g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition } : g.Fanuc.Joints.Select(a => a.CurrentPosition).ToArray(),
                    targets = g.Scara != null ? new[] { g.Scara.Axis1.TargetPosition, g.Scara.Axis2.TargetPosition, g.Scara.Axis3.TargetPosition, g.Scara.Axis4.TargetPosition } : g.Fanuc.Joints.Select(a => a.TargetPosition).ToArray(),
                    q.State, q.StepIndex, q.GripCount, q.ReleaseCount, held = held != null ? held.GetInstanceID() : 0,
                    position = held != null ? V(held.Body.position) : null, rotation = held != null ? Q(held.Body.rotation) : null };
            }) });
    }
    private static string Full(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.LastAcknowledgedFaultReason, trackerFault = t.IsFaulted,
            trackerCause = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason, data = Data(l) });
    }
    private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }
        .SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))).Replace("-", ""); }
    private static void CheckSources(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification changed code/scene/assets.");
    private static async Task Ticks(int n) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= n * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
