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

public static class VerifyPickFaultRecovery
{
    public static async Task<string> Reproduce()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var g = l.Scara.GetComponent<RobotGripper>();
        var q = g.GetComponent<RobotWaypointSequence>(); var original = q.Waypoints; var copy = UnityEngine.Object.Instantiate(original); float scale = Time.timeScale;
        try
        {
            Time.timeScale = .1f; SetWaypoints(q, copy);
            var lab = g.GetComponent<RobotGripperLab>(); Require(lab.TryPrepare(out var e), e);
            Require(q.TryStartPickPlace(g, lab.TrialItem, out e), e); q.Pause(); Require(c.TryTakeControl(q, out e), e);
            Require(c.TryExecute(CellRunCommand.Start, out e), e);
            SetPosition(copy, "retreat", original.Retreat.PositionMetres + Vector3.right * .01f);
            await Until(() => q.State == State.Faulted && c.IsFaulted);
            SetPosition(copy, "retreat", original.Retreat.PositionMetres); await Ticks(2);
            var allowed = c.CanExecute(CellRunCommand.Reset, out e);
            var report = new { reproduced = !allowed && q.IsPickPlace && q.State == State.Faulted && c.IsFaulted, error = e,
                q.StepIndex, q.GripCount, q.ReleaseCount, owned = c.ControlledSequence == q };
            Write("pick-fault-recovery-before.json", report); return JsonConvert.SerializeObject(report);
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetWaypoints(q, original); UnityEngine.Object.Destroy(copy); }
    }

    public static async Task<string> Restored(int robot)
    {
        Require(robot == 0 || robot == 1, "Unknown robot.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = (robot == 0 ? l.Scara.gameObject : l.Fanuc.gameObject).GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>();
        var source = q.Waypoints; var copy = UnityEngine.Object.Instantiate(source); var station = robot == 0 ? l.HandoffPoint : l.OutputA;
        var workStation = robot == 0 ? l.InputStop : l.HandoffPoint; var originalWorkStation = workStation.position;
        var originalStation = station.position; float scale = Time.timeScale; var hashes = Sources(); int rejected = 0;
        try
        {
            Time.timeScale = 5; var item = await Prepare(l, g, copy); int instance = item.GetInstanceID();
            Require(c.TryTakeControl(q, out var e), e); var retreat = copy.Retreat.PositionMetres; string previousArchive = q.LastAcknowledgedFaultReason;
            for (int phase = 0; phase < 3; phase++)
            {
                Time.timeScale = 5; Require(c.TryExecute(CellRunCommand.Start, out e), e);
                if (phase == 0)
                {
                    // The slot and original item still agree, but the actual grip misses.
                    // Check the real action rejection before any physical/logical claim.
                    var position = item.Body.position; workStation.position += Vector3.right * .01f; item.Body.position += Vector3.right * .01f;
                    await Until(() => q.State == State.Faulted && c.IsFaulted);
                    Require(q.StepIndex == 1 && q.GripCount == 0 && q.ReleaseCount == 0 && g.HeldItem == null && item.Owner == null && q.LastError.Contains("접촉 기준"),
                        q.LastError ?? "Rejected grip changed action counts or claimed the item.");
                    workStation.position = originalWorkStation; item.Body.position = position;
                }
                if (phase == 1) await Until(() => q.GripCount == 1 && q.StepIndex >= 2 || c.IsFaulted);
                if (phase < 2)
                {
                    SetPosition(copy, "retreat", retreat + Vector3.right * .01f);
                    if (phase == 1) await Until(() => q.State == State.Faulted && c.IsFaulted);
                    Require(q.GripCount == (phase == 0 ? 0 : 1) && q.ReleaseCount == 0, "Fault fixture crossed the action boundary.");
                    if (phase == 1) { t.enabled = false; t.enabled = true; }
                    string before = Full(l);
                    Require(!c.TryExecute(CellRunCommand.Reset, out e) && !string.IsNullOrEmpty(e) && Full(l) == before,
                        "Changed program cleared some latches or changed action/data history."); rejected++;
                    SetPosition(copy, "retreat", retreat);
                }
                else
                {
                    // An actual release attempt must fail before its logical/physical commit.
                    // This deliberately moves only the Play fixture station, not the saved target.
                    station.position += Vector3.right * .01f;
                    await Until(() => q.State == State.Faulted && c.IsFaulted);
                    Require(q.StepIndex == 3 && q.GripCount == 1 && q.ReleaseCount == 0 && g.HeldItem == item && item.Owner == g && q.LastError.Contains("놓기 지점"),
                        q.LastError ?? "Release rejection did not preserve the original holding/action counts.");
                    station.position -= Vector3.right * .01f;
                }
                Require(q.LastAcknowledgedFaultReason == previousArchive, "A new fault erased the previous acknowledgement.");
                q.enabled = false; q.enabled = true; c.enabled = false; c.enabled = true;
                Time.timeScale = 1; await Ticks(2);
                Require(q.State == State.Faulted && c.IsFaulted, "Program/physical/component restoration auto-cleared a fault.");
                string full = Full(l);
                for (int i = 0; i < 100; i++) Require(c.CanExecute(CellRunCommand.Reset, out e), e);
                Require(Full(l) == full, "Recovery queries mutated action history or ownership.");
                var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
                var label = robot == 0 ? "cell-run-scara" : "cell-run-fanuc";
                Require(window.rootVisualElement.Q<Button>("cell-run-reset").enabledInHierarchy && window.rootVisualElement.Q<Label>(label).text.Contains("경로 고장 유지"), "Recoverable action phase missing from UI.");
                Acknowledge(l, q); previousArchive = q.LastAcknowledgedFaultReason;
                string data = Data(l); await Ticks(3);
                Require(Data(l) == data && q.State == State.Paused && c.State == CellRunState.Stopped && item.GetInstanceID() == instance && item.ProductId == "P-000001",
                    "Acknowledgement auto-started, changed identity/data or crossed an action boundary.");
                window.RefreshCell(); Require(window.rootVisualElement.Q<Label>(label).text.Contains("최근 경로 명시 해제") && !window.rootVisualElement.Q<Button>("cell-run-reset").enabledInHierarchy,
                    "Acknowledged action phase/duplicate reset state missing from UI.");
                full = Full(l);
                Require(!c.TryExecute(CellRunCommand.Reset, out _) && !q.TryResume(out _) && !q.TryStartPickPlace(g, item, out _) && !g.TryRelease(out _) && Full(l) == full,
                    "Duplicate reset/individual actions bypassed the adopted program."); rejected += 4;
            }
            Time.timeScale = 5; Require(c.TryExecute(CellRunCommand.Start, out e), e);
            await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1 && g.HeldItem == null && item.Owner == null &&
                item.GetInstanceID() == instance && item.ProductId == "P-000001" && t.TryGet("P-000001", out var record) &&
                record.Location == (robot == 0 ? ProductLocation.Handoff : ProductLocation.OutputA), q.LastError ?? c.FaultReason ?? "Duplicate/missing grip or release after explicit resume.");
            Require(t.Count == 3 && t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B" }), "Recovery changed registration/FIFO.");
            CheckSources(hashes);
            var report = new { passed = true, robot, verifiedUtc = DateTime.UtcNow, pathAcknowledgements = 3, eligibilityQueriesPreserved = 300,
                rejectedRequestsPreserved = rejected, rejectedAutomaticGrips = 1, rejectedAutomaticReleases = 1, automaticGrips = q.GripCount, automaticReleases = q.ReleaseCount,
                registeredProducts = t.Count, completedProducts = robot == 0 ? 0 : 1, physicallyConfirmedResidual = robot == 0 ? 3 : 2, unconfirmedProducts = 0,
                sameProductInstanceAndId = true, motionCommandsAtAcknowledgement = 0, sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0,
                fixtureReleaseElevationMetres = .07f, scope = "Adopted pick/place recovery before grip, while holding, and after a rejected release. Raised Play-only release fixture; not the standard same-height production route.", hashes };
            Write("pick-fault-recovery-restored-" + robot + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.robot, report.pathAcknowledgements, report.automaticGrips, report.automaticReleases, report.rejectedAutomaticReleases, report.registeredProducts, report.completedProducts });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); station.position = originalStation; workStation.position = originalWorkStation; SetWaypoints(q, source); UnityEngine.Object.Destroy(copy); }
    }

    public static async Task<string> Trial(int robot)
    {
        Require(robot == 0 || robot == 1, "Unknown robot.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = (robot == 0 ? l.Scara.gameObject : l.Fanuc.gameObject).GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>();
        var source = q.Waypoints; var copy = UnityEngine.Object.Instantiate(source); float scale = Time.timeScale; var hashes = Sources();
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); string records = JsonConvert.SerializeObject(t.Snapshot());
            SetWaypoints(q, copy); var lab = g.GetComponent<RobotGripperLab>(); Require(lab.TryPrepare(out var e), e); var item = lab.TrialItem;
            Require(!item.IsTracked && q.TryStartPickPlace(g, item, out e), e); q.Pause(); Require(c.TryTakeControl(q, out e), e);
            Require(c.TryExecute(CellRunCommand.Start, out e), e); await Until(() => q.GripCount == 1 && q.StepIndex >= 2 || c.IsFaulted);
            SetPosition(copy, "retreat", source.Retreat.PositionMetres + Vector3.right * .01f); await Until(() => q.State == State.Faulted && c.IsFaulted);
            SetPosition(copy, "retreat", source.Retreat.PositionMetres); Time.timeScale = 1; await Ticks(2);
            Acknowledge(l, q); string before = Data(l); await Ticks(3);
            Require(Data(l) == before && q.State == State.Paused && c.State == CellRunState.Stopped && g.HeldItem == item && item.Owner == g,
                "Untracked trial recovery moved, dropped or restarted the saved holding.");
            Time.timeScale = 5; Require(c.TryExecute(CellRunCommand.Start, out e), e); await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1 && item.Owner == null && g.HeldItem == null &&
                JsonConvert.SerializeObject(t.Snapshot()) == records && !t.IsFaulted, q.LastError ?? c.FaultReason ?? "Trial recovery repeated an action or changed registered products.");
            CheckSources(hashes);
            var report = new { passed = true, robot, verifiedUtc = DateTime.UtcNow, pathAcknowledgements = 1, automaticGrips = 1, automaticReleases = 1,
                registeredProducts = 1, completedRegisteredProducts = 0, physicallyConfirmedResidual = 1, unconfirmedProducts = 0,
                motionCommandsAtAcknowledgement = 0, sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0,
                scope = "Existing untracked trial pick/place and production waypoint asset, registered input record unchanged.", hashes };
            Write("pick-fault-recovery-trial-" + robot + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.robot, report.pathAcknowledgements, report.automaticGrips, report.automaticReleases, report.registeredProducts });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetWaypoints(q, source); UnityEngine.Object.Destroy(copy); }
    }

    public static async Task<string> Blocked(int scenario)
    {
        Require(scenario >= 0 && scenario <= 5, "Unknown blocked scenario.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = l.Scara.GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>(); var source = q.Waypoints;
        var copy = UnityEngine.Object.Instantiate(source); var releasePosition = l.HandoffPoint.position; var inputPosition = l.InputStop.position;
        float scale = Time.timeScale, link = 0; var hashes = Sources(); bool repaired = false;
        try
        {
            Time.timeScale = 5; var item = await Prepare(l, g, copy); var retreat = copy.Retreat.PositionMetres;
            if (scenario != 5) Require(c.TryTakeControl(q, out var e), e);
            Time.timeScale = scenario == 1 || scenario == 5 ? .1f : 5f;
            if (scenario == 5) Require(q.TryResume(out var e), e); else Require(c.TryExecute(CellRunCommand.Start, out var e), e);
            if (scenario != 1 && scenario != 5) await Until(() => q.GripCount == 1 && q.StepIndex >= 2 || c.IsFaulted);
            SetPosition(copy, "retreat", retreat + Vector3.right * .01f); await Until(() => q.State == State.Faulted && c.IsFaulted);
            SetPosition(copy, "retreat", retreat); t.enabled = false; t.enabled = true; Time.timeScale = 1;
            var itemPosition = item.Body.position;
            if (scenario == 0) { item.gameObject.SetActive(false); item.gameObject.SetActive(true); }
            else if (scenario == 1)
            {
                // Physical slot still agrees with the registry, but the saved planned grip does not.
                item.Body.position += Vector3.right * .01f; l.InputStop.position += Vector3.right * .01f;
                Require(c.CanExecute(CellRunCommand.Reset, out _), "Fixture must reach final planned-grip validation.");
            }
            else if (scenario == 2)
            { var so = new SerializedObject(l.Scara); var f = so.FindProperty("fkLink1Metres"); link = f.floatValue; f.floatValue = link + .01f; so.ApplyModifiedPropertiesWithoutUndo(); }
            else if (scenario == 3) SetPosition(copy, "retreat", retreat + Vector3.right * .01f);
            else if (scenario == 4)
            {
                var rotation = item.Body.rotation; UnityEngine.Object.Destroy(item.gameObject); await Ticks(2);
                var replacement = UnityEngine.Object.Instantiate(l.ProductPrefab); replacement.Body.position = itemPosition; replacement.Body.rotation = rotation;
                Require(!replacement.IsTracked && !t.TryRegister(replacement, "P-000001", ProductDestination.A, out _), "Replacement rebound the lost original ID.");
            }
            await Ticks(2); string before = Full(l);
            Require(!c.TryExecute(CellRunCommand.Reset, out var error) && !string.IsNullOrEmpty(error) && Full(l) == before && q.State == State.Faulted && c.IsFaulted && t.IsFaulted,
                "Failed action/physical validation partially acknowledged one of the three faults.");
            if (scenario == 1 || scenario == 2 || scenario == 3)
            {
                if (scenario == 1) { item.Body.position = itemPosition; l.InputStop.position = inputPosition; }
                else if (scenario == 2) { var so = new SerializedObject(l.Scara); so.FindProperty("fkLink1Metres").floatValue = link; so.ApplyModifiedPropertiesWithoutUndo(); }
                else SetPosition(copy, "retreat", retreat);
                await Ticks(2); Require(q.State == State.Faulted && c.IsFaulted && t.IsFaulted, "Repair auto-cleared a fault.");
                Acknowledge(l, q); repaired = true;
            }
            else Require(!c.CanExecute(CellRunCommand.Reset, out _), "Lost/unadopted action history became recoverable.");
            Require(t.Count == 3 && t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B" }), "Negative action recovery changed totals/FIFO.");
            if (scenario == 0)
                Require(item.Owner == null && g.HeldItem == null && t.ScaraProductId == "P-000001" && q.CycleItem == item,
                    "Lost Owner was normalized into confirmed residual/holding.");
            if (scenario == 4)
                Require(!t.TryGetItem("P-000001", out _) && t.ScaraProductId == "P-000001" && g.HeldItem == null && q.CycleItem == null,
                    "Missing original instance was rebound or normalized into confirmed residual.");
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, rejectedRequestsPreserved = 1,
                noPartialLatchClearOnFailure = true, repairedThenExplicitlyAcknowledged = repaired, unconfirmedProducts = scenario == 0 || scenario == 4 ? 1 : 0,
                finalPlannedGripRejected = scenario == 1, motionCommandsAtAcknowledgement = 0, automaticRecovery = false,
                sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, hashes };
            Write("pick-fault-recovery-blocked-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.noPartialLatchClearOnFailure, report.repairedThenExplicitlyAcknowledged, report.unconfirmedProducts });
        }
        finally
        {
            if (link != 0) { var so = new SerializedObject(l.Scara); so.FindProperty("fkLink1Metres").floatValue = link; so.ApplyModifiedPropertiesWithoutUndo(); }
            Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); l.HandoffPoint.position = releasePosition; l.InputStop.position = inputPosition;
            SetWaypoints(q, source); UnityEngine.Object.Destroy(copy);
        }
    }

    // Keep the existing four-step format and production assets intact. Its release target
    // must be at/above approach height, so this isolated Play fixture raises only that station.
    private static async Task<GrippableItem> Prepare(LineWatchCellLayout l, RobotGripper g, RobotWaypointSet copy)
    {
        var t = l.GetComponent<CellProductTracker>(); await Until(() => t.Count == 1);
        AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f); Require(t.TryGetItem("P-000001", out var item), "Initial product missing.");
        bool scara = g.Scara != null;
        if (!scara)
        {
            var sg = l.Scara.GetComponent<RobotGripper>(); await Move(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
            await Move(sg, item, l.HandoffPoint); Require(sg.TryRelease(out e), e);
            var retreat = l.ScaraHandoffWaypoints.Retreat;
            Require(l.Scara.TryMoveTcp(retreat.PositionMetres, Yaw(retreat.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out e), e); await Until(() => sg.IsStopped);
        }
        var release = scara ? l.HandoffPoint : l.OutputA; release.position += Vector3.up * .07f;
        var work = TcpGoal(g, item, scara ? l.InputStop : l.HandoffPoint); var goal = TcpGoal(g, item, release);
        SetTarget(copy, "work", work); SetTarget(copy, "approach", new Pose(work.position + Vector3.up * .03f, work.rotation)); SetTarget(copy, "retreat", goal);
        Require(copy.TryValidate(out var error), error); var q = g.GetComponent<RobotWaypointSequence>(); SetWaypoints(q, copy);
        Require(q.TryStartPickPlace(g, item, out error), error); q.Pause(); return item;
    }
    private static Pose TcpGoal(RobotGripper g, GrippableItem item, Transform station)
    {
        var point = item.GetGripPoint(g.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc);
        var rotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = station.TransformPoint(point.localPosition) - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        return new Pose(g.RobotFrame.InverseTransformPoint(position), Quaternion.Inverse(g.RobotFrame.rotation) * rotation);
    }
    private static void SetTarget(RobotWaypointSet set, string name, Pose pose)
    {
        var so = new SerializedObject(set); var f = so.FindProperty(name); f.FindPropertyRelative("positionMetres").vector3Value = pose.position;
        f.FindPropertyRelative("rotation").quaternionValue = pose.rotation; so.ApplyModifiedPropertiesWithoutUndo();
    }
    private static async Task Move(RobotGripper g, GrippableItem item, Transform station)
    {
        var goal = TcpGoal(g, item, station); string error;
        bool ok = g.Scara != null ? g.Scara.TryMoveTcp(goal.position, Yaw(goal.rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out error)
            : g.Fanuc.TryMoveTcp(goal.position, goal.rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error);
        Require(ok, error); await Until(() => g.IsStopped); await Ticks(2);
    }
    private static void AddWaiting(LineWatchCellLayout l, string id, float offset)
    {
        var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset;
        Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e);
    }
    private static void Acknowledge(LineWatchCellLayout l, RobotWaypointSequence q)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); var data = Data(l);
        string cellCause = c.FaultReason, pathCause = q.LastError, trackerCause = t.FaultReason, archive = t.LastAcknowledgedFaultReason; bool trackingFault = t.IsFaulted;
        Require(c.TryExecute(CellRunCommand.Reset, out var error), error);
        Require(c.State == CellRunState.Stopped && q.State == State.Paused && !t.IsFaulted && c.FaultReason == null && q.LastError == null && t.FaultReason == null &&
            c.LastAcknowledgedFaultReason == cellCause && q.LastAcknowledgedFaultReason == pathCause &&
            t.LastAcknowledgedFaultReason == (trackingFault ? trackerCause : archive) && Data(l) == data,
            "Acknowledgement dispatched, crossed an action boundary or lost independent cause/data history.");
    }
    private static string Data(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        return JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(), slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            physical = t.Snapshot().Select(r => new { r.Id, instance = t.TryGetItem(r.Id, out var item) ? item.GetInstanceID() : 0, owner = item != null && item.Owner != null ? item.Owner.GetInstanceID() : 0 }),
            c.HasControlledSequence, controlled = c.ControlledSequence != null ? c.ControlledSequence.GetInstanceID() : 0,
            robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() }.Select(g => {
                var q = g.GetComponent<RobotWaypointSequence>(); var held = g.HeldItem;
                return new { version = g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion,
                    current = g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition } : g.Fanuc.Joints.Select(a => a.CurrentPosition).ToArray(),
                    targets = g.Scara != null ? new[] { g.Scara.Axis1.TargetPosition, g.Scara.Axis2.TargetPosition, g.Scara.Axis3.TargetPosition, g.Scara.Axis4.TargetPosition } : g.Fanuc.Joints.Select(a => a.TargetPosition).ToArray(),
                    q.StepIndex, q.GripCount, q.ReleaseCount, q.IsPickPlace, q.LastArrivedStepIndex, cycleItem = q.CycleItem != null ? q.CycleItem.GetInstanceID() : 0,
                    arrivalPosition = V(q.LastArrivalPose.position), arrivalRotation = Q(q.LastArrivalPose.rotation), held = held != null ? held.GetInstanceID() : 0,
                    position = held != null ? V(held.Body.position) : null, rotation = held != null ? Q(held.Body.rotation) : null };
            }) });
    }
    private static string Full(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.LastAcknowledgedFaultReason, trackerFault = t.IsFaulted,
            trackerCause = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason,
            paths = new[] { l.Scara.GetComponent<RobotWaypointSequence>(), l.Fanuc.GetComponent<RobotWaypointSequence>() }.Select(q => new { q.State, q.LastError, q.LastAcknowledgedFaultReason }), data = Data(l) });
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
    private static void SetPosition(RobotWaypointSet set, string name, Vector3 position)
    { var so = new SerializedObject(set); so.FindProperty(name).FindPropertyRelative("positionMetres").vector3Value = position; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static async Task Ticks(int n) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= n * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
