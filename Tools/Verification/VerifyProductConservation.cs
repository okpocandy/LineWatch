using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class VerifyProductConservation
{
    private sealed class Inventory
    {
        public int Registered, CompletedA, CompletedB, Removed, ConfirmedLost, VerifiedRemaining, Unresolved;
        public bool LogicalConsistent, EquationBalanced, PhysicalReconciled, TrackingHealthy;
        public bool TrackerFaulted;
        public readonly List<string> UnresolvedIds = new List<string>();
        public readonly List<string> Issues = new List<string>();
        public readonly List<object> Rows = new List<object>();
    }

    public static async Task<object> Normal()
    {
        var layout = Layout(); var tracker = layout.GetComponent<CellProductTracker>();
        await Until(() => tracker.Count == 1 || tracker.IsFaulted);
        var hashes = Hashes(); var samples = new List<object>(); int rejected = 0, checkpoints = 0;
        var sg = layout.Scara.GetComponent<RobotGripper>(); var fg = layout.Fanuc.GetComponent<RobotGripper>();
        Require(tracker.TryGetItem("P-000001", out var first), "Initial product missing.");
        float scale = Time.timeScale;
        void Check(string name)
        {
            var audit = Audit(layout, tracker); Require(audit.TrackingHealthy, "Inventory mismatch at " + name + ": " + string.Join(";", audit.Issues));
            samples.Add(new { name, audit }); checkpoints++;
        }
        string State() => JsonConvert.SerializeObject(new { tracker.IsFaulted, tracker.InputProductId, tracker.ScaraProductId,
            tracker.HandoffProductId, tracker.FanucProductId, waiting = tracker.WaitingSnapshot(), audit = Audit(layout, tracker) });
        async Task Reject(Func<bool> request, string name)
        {
            string before = State(); Require(!request(), "Invalid request accepted: " + name);
            await Frames(3); Require(before == State(), "Rejected request changed deferred state: " + name); Check(name); rejected++;
        }
        async Task<GrippableItem> Register(string id, ProductDestination destination, float offset)
        {
            var item = UnityEngine.Object.Instantiate(layout.ProductPrefab);
            item.Body.position = layout.InputStop.position + Vector3.right * offset; item.Body.rotation = Quaternion.identity;
            Require(tracker.TryRegister(item, id, destination, out var error), error); await Frames(2); Check("register " + id); return item;
        }
        async Task Feed(GrippableItem item)
        {
            item.Body.position = layout.InputStop.position; item.Body.rotation = layout.InputStop.rotation;
            Require(tracker.TryFeedNext(out var record, out var error), error); Require(record.Id == item.ProductId, "FIFO feed reordered.");
            await Frames(2); Check("feed " + item.ProductId);
        }
        async Task Grip(RobotGripper g, GrippableItem item, Transform station)
        {
            await Move(g, item, station); Require(g.TryGrip(item, out var error), error);
            await Frames(2); Check("grip " + item.ProductId);
        }
        async Task Release(RobotGripper g, GrippableItem item, Transform station)
        {
            await Move(g, item, station); Require(g.TryRelease(out var error), error);
            await Frames(2); Check("release " + item.ProductId);
        }
        async Task Remove(GrippableItem item, ProductLocation expected)
        {
            string id = item.ProductId; Require(tracker.TryRemove(id, expected, out var error), error);
            await Frames(3); Require(!tracker.TryGetItem(id, out _), "Removed object remains live."); Check("remove " + id);
        }
        try
        {
            Time.timeScale = 10; Check("bootstrap");
            var waitingRemoval = await Register("WAIT-REMOVE", ProductDestination.A, .55f);
            var inputRemoval = await Register("INPUT-REMOVE", ProductDestination.B, .60f);
            var handoffRemoval = await Register("HANDOFF-REMOVE", ProductDestination.A, .65f);
            var second = await Register("P-B", ProductDestination.B, .70f);
            var remaining = await Register("REMAIN", ProductDestination.A, .75f);
            await Reject(() => tracker.TryFeedNext(out _, out _), "occupied input preserves FIFO");
            Time.timeScale = 0;
            await Reject(() => tracker.TryRemove(first.ProductId, ProductLocation.Input, out _), "paused clock preserves product");
            Time.timeScale = 10;
            await Remove(waitingRemoval, ProductLocation.Waiting);
            Require(tracker.WaitingSnapshot().SequenceEqual(new[] { "INPUT-REMOVE", "HANDOFF-REMOVE", "P-B", "REMAIN" }), "Waiting removal changed order.");
            await Grip(sg, first, layout.InputStop); await Move(sg, first, layout.InputStop, .07f);
            await Reject(() => tracker.TryRemove(first.ProductId, ProductLocation.Scara, out _), "held removal rejected");
            await Feed(inputRemoval); await Remove(inputRemoval, ProductLocation.Input);
            await Feed(handoffRemoval);
            await Release(sg, first, layout.HandoffPoint); await Move(sg, first, layout.HandoffPoint, .07f);
            await Grip(fg, first, layout.HandoffPoint);
            await Release(fg, first, layout.OutputA); await Move(fg, first, layout.OutputA, .06f);
            await Reject(() => fg.TryRelease(out _), "duplicate release does not count twice");
            await Reject(() => tracker.TryRemove(first.ProductId, ProductLocation.OutputA, out _), "completion cannot become removal");
            await Grip(sg, handoffRemoval, layout.InputStop);
            await Release(sg, handoffRemoval, layout.HandoffPoint); await Move(sg, handoffRemoval, layout.HandoffPoint, .07f);
            await Remove(handoffRemoval, ProductLocation.Handoff);
            await Feed(second);
            await Grip(sg, second, layout.InputStop);
            await Release(sg, second, layout.HandoffPoint); await Move(sg, second, layout.HandoffPoint, .07f);
            await Grip(fg, second, layout.HandoffPoint);
            await Release(fg, second, layout.OutputB); await Move(fg, second, layout.OutputB, .06f);
            await Feed(remaining);
            var beforeDespawn = Audit(layout, tracker);
            Require(beforeDespawn.Registered == 6 && beforeDespawn.CompletedA == 1 && beforeDespawn.CompletedB == 1 && beforeDespawn.Removed == 3 &&
                beforeDespawn.VerifiedRemaining == 1 && beforeDespawn.Unresolved == 0 && beforeDespawn.TrackingHealthy, "Final normal balance failed.");
            UnityEngine.Object.Destroy(first.gameObject); UnityEngine.Object.Destroy(second.gameObject); await Frames(3);
            Check("completed objects despawned; history retained");
            var final = Audit(layout, tracker);
            Require(final.CompletedA == 1 && final.CompletedB == 1 && final.Removed == 3 && final.VerifiedRemaining == 1 && final.Unresolved == 0 &&
                tracker.InputProductId == "REMAIN" && tracker.ScaraProductId == null && tracker.HandoffProductId == null && tracker.FanucProductId == null,
                "Despawn changed completion totals or occupancy.");
            Require(hashes.All(p => Hash(p.Key) == p.Value), "Saved source changed.");
            var report = new { passed = true, final, checkpoints, rejectedRequestsPreserved = rejected,
                completedObjectDespawnPreservesHistory = true, explicitRemovalLocations = new[] { "Waiting", "Input", "Handoff" },
                sourceFiles = hashes.Count, sourceAssetsUnchanged = true, hashes, samples, directButtonClicks = 0,
                scope = "Registered-product conservation and object/owner/slot reconciliation through explicit APIs. No automatic cell state machine, collision/force or 100-item integration claim." };
            Write("conservation-normal.json", report);
            return JsonConvert.SerializeObject(new { report.passed, final.Registered, final.CompletedA, final.CompletedB, final.Removed, final.ConfirmedLost, final.VerifiedRemaining,
                final.Unresolved, checkpoints, report.rejectedRequestsPreserved, report.completedObjectDespawnPreservesHistory, report.sourceFiles, report.sourceAssetsUnchanged });
        }
        finally { Time.timeScale = scale; sg.Stop(); fg.Stop(); }
    }

    public static async Task<object> Faults(int scenario)
    {
        var layout = Layout(); var tracker = layout.GetComponent<CellProductTracker>();
        await Until(() => tracker.Count == 1 || tracker.IsFaulted);
        Require(!tracker.IsFaulted, "Fresh healthy tracker required.");
        Require(tracker.TryGetItem("P-000001", out var item), "Initial product missing.");
        var hashes = Hashes(); var sg = layout.Scara.GetComponent<RobotGripper>(); float scale = Time.timeScale;
        try
        {
            Time.timeScale = 10;
            if (scenario >= 2)
            {
                await Move(sg, item, layout.InputStop); Require(sg.TryGrip(item, out var error), error);
                Require(layout.Scara.MoveJoint(0, layout.Scara.Axis1.CurrentPosition + 10) && !Rest(sg), "Active stop fixture failed.");
            }
            if (scenario == 0) UnityEngine.Object.Destroy(item.gameObject);
            else if (scenario == 1) item.Body.position += Vector3.up * .02f;
            else if (scenario == 2) sg.enabled = false;
            else if (scenario == 3) tracker.enabled = false;
            else throw new ArgumentException("Unknown scenario.");
            await Frames(3);
            var faultAudit = Audit(layout, tracker);
            Require(tracker.IsFaulted && !faultAudit.TrackingHealthy && faultAudit.EquationBalanced && faultAudit.LogicalConsistent && Rest(sg), "Fault handling or logical count failed.");
            if (scenario < 3) Require(faultAudit.Unresolved == 1 && faultAudit.VerifiedRemaining == 0 && !faultAudit.PhysicalReconciled,
                "Unresolved product was counted as verified remaining.");
            else Require(faultAudit.Unresolved == 0 && faultAudit.VerifiedRemaining == 1 && faultAudit.PhysicalReconciled,
                "Preserved held item was incorrectly classified as lost.");
            if (scenario == 1) { item.Body.position = layout.InputStop.position; item.Body.rotation = layout.InputStop.rotation; }
            if (scenario == 2) sg.enabled = true;
            if (scenario == 3) tracker.enabled = true;
            await Frames(3);
            var afterRestoration = Audit(layout, tracker);
            Require(tracker.IsFaulted && !afterRestoration.TrackingHealthy, "Physical restoration automatically authorized operation.");
            string records = JsonConvert.SerializeObject(tracker.Snapshot());
            Require(!tracker.TryRemove("P-000001", scenario < 2 ? ProductLocation.Input : ProductLocation.Scara, out _) &&
                records == JsonConvert.SerializeObject(tracker.Snapshot()), "Faulted request changed last-known records.");
            if (scenario == 1 || scenario == 3) Require(afterRestoration.PhysicalReconciled && afterRestoration.Unresolved == 0,
                "Restored physical state should reconcile while fault remains latched.");
            Require(hashes.All(p => Hash(p.Key) == p.Value), "Sources changed during fault verification.");
            var report = new { passed = true, scenario, faultReason = tracker.FaultReason, faultAudit, afterRestoration,
                unresolvedNotCountedAsNormalRemaining = scenario < 3, faultRemainsLatched = true, sourceAssetsUnchanged = true, hashes, directButtonClicks = 0 };
            Write("conservation-fault-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, scenario, unresolvedAtFault = faultAudit.Unresolved,
                verifiedRemainingAtFault = faultAudit.VerifiedRemaining, physicalReconciledAfterRestoration = afterRestoration.PhysicalReconciled,
                report.faultRemainsLatched, report.sourceAssetsUnchanged });
        }
        finally { Time.timeScale = scale; sg.Stop(); }
    }

    public static object ConfirmedLossAudit()
    {
        var layout = Layout(); var audit = Audit(layout, layout.GetComponent<CellProductTracker>());
        Require(audit.ConfirmedLost > 0 && audit.PhysicalReconciled && audit.EquationBalanced, "Loss-inclusive conservation audit failed: " + string.Join(";", audit.Issues));
        var report = new { passed = true, audit.Registered, audit.CompletedA, audit.CompletedB, audit.Removed, audit.ConfirmedLost,
            audit.VerifiedRemaining, audit.Unresolved, audit.PhysicalReconciled, audit.TrackingHealthy, audit.TrackerFaulted, audit.Rows };
        Write("conservation-confirmed-loss.json", report); return report;
    }

    private static Inventory Audit(LineWatchCellLayout layout, CellProductTracker tracker)
    {
        var result = new Inventory { Registered = tracker.Count, TrackerFaulted = tracker.IsFaulted };
        var records = tracker.Snapshot(); var waiting = tracker.WaitingSnapshot(); bool logical = true;
        void LogicalIssue(string reason) { logical = false; result.Issues.Add(reason); }
        if (records.Count != tracker.Count || records.Select(r => r.Id).Distinct(StringComparer.Ordinal).Count() != tracker.Count) LogicalIssue("record count/ID mismatch");
        if (waiting.Distinct(StringComparer.Ordinal).Count() != waiting.Count || !waiting.OrderBy(id => id, StringComparer.Ordinal).SequenceEqual(
            records.Where(r => r.Location == ProductLocation.Waiting).Select(r => r.Id).OrderBy(id => id, StringComparer.Ordinal))) LogicalIssue("FIFO membership mismatch");
        foreach (var slot in new[] { (ProductLocation.Input, tracker.InputProductId), (ProductLocation.Scara, tracker.ScaraProductId),
            (ProductLocation.Handoff, tracker.HandoffProductId), (ProductLocation.Fanuc, tracker.FanucProductId) })
        {
            var owners = records.Where(r => r.Location == slot.Item1).ToArray();
            if (owners.Length > 1 || (owners.Length == 0 ? slot.Item2 != null : owners[0].Id != slot.Item2)) LogicalIssue("slot mismatch: " + slot.Item1);
        }
        var sg = layout.Scara.GetComponent<RobotGripper>(); var fg = layout.Fanuc.GetComponent<RobotGripper>();
        var identities = new HashSet<int>();
        foreach (var record in records)
        {
            bool terminal = record.Location == ProductLocation.OutputA || record.Location == ProductLocation.OutputB || record.Location == ProductLocation.Removed;
            if (record.Location == ProductLocation.OutputA) { result.CompletedA++; if (record.Destination != ProductDestination.A) LogicalIssue(record.Id + ":wrong output A"); }
            if (record.Location == ProductLocation.OutputB) { result.CompletedB++; if (record.Destination != ProductDestination.B) LogicalIssue(record.Id + ":wrong output B"); }
            if (record.Location == ProductLocation.Removed) result.Removed++;
            bool exists = tracker.TryGetItem(record.Id, out var item); string reason = null;
            if (exists && (!identities.Add(item.GetInstanceID()) || item.ProductId != record.Id || item.ProductTracker != tracker)) reason = "identity mismatch";
            if (record.Location == ProductLocation.Lost)
            {
                if (exists || string.IsNullOrWhiteSpace(record.LossReason) || !record.LostFrom.HasValue ||
                    record.LostFrom == ProductLocation.OutputA || record.LostFrom == ProductLocation.OutputB || record.LostFrom == ProductLocation.Removed || record.LostFrom == ProductLocation.Lost)
                    reason = "loss receipt/physical destruction mismatch";
                if (reason == null) result.ConfirmedLost++;
                else { result.Unresolved++; result.UnresolvedIds.Add(record.Id); }
            }
            else if (terminal)
            {
                if (exists && item.Owner != null) reason = "terminal record is physically held";
            }
            else
            {
                if (!exists) reason = "physical item missing";
                else if (!item.IsReady) reason = "physical item unavailable";
                else
                {
                    var expected = record.Location == ProductLocation.Scara ? sg : record.Location == ProductLocation.Fanuc ? fg : null;
                    if (item.Owner != expected || expected != null && expected.HeldItem != item) reason = "physical owner mismatch";
                    var point = record.Location == ProductLocation.Input ? layout.InputStop : record.Location == ProductLocation.Handoff ? layout.HandoffPoint : null;
                    if (point != null && (Vector3.Distance(item.Body.position, point.position) > RobotGripper.PositionToleranceMetres ||
                        RobotGripper.RotationError(item.Body.rotation, point.rotation) > RobotGripper.RotationToleranceDegrees)) reason = "recorded station pose mismatch";
                }
                if (reason == null) result.VerifiedRemaining++;
                else { result.Unresolved++; result.UnresolvedIds.Add(record.Id); }
            }
            if (reason != null) result.Issues.Add(record.Id + ":" + reason);
            result.Rows.Add(new { record.Id, destination = record.Destination.ToString(), location = record.Location.ToString(), record.LostFrom, record.LossReason, exists,
                owner = exists && item.Owner != null ? item.Owner.name : "none", position = exists ? V(item.Body.position) : null,
                rotation = exists ? Q(item.Body.rotation) : null, reason });
        }
        result.LogicalConsistent = logical;
        result.EquationBalanced = result.Registered == result.CompletedA + result.CompletedB + result.Removed + result.ConfirmedLost + result.VerifiedRemaining + result.Unresolved;
        result.PhysicalReconciled = logical && result.EquationBalanced && result.Unresolved == 0 && result.Issues.Count == 0;
        result.TrackingHealthy = result.PhysicalReconciled && !tracker.IsFaulted && tracker.isActiveAndEnabled;
        return result;
    }

    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh unpaused LineWatch Play Mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>(); Require(layout != null, "Open LineWatch."); return layout;
    }
    private static async Task Move(RobotGripper g, GrippableItem item, Transform station, float lift = 0)
    {
        var point = item.GetGripPoint(g.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc);
        var rotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = station.TransformPoint(point.localPosition) + Vector3.up * lift - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        string error;
        bool ok = g.Scara != null ? g.Scara.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position),
            Vector3.SignedAngle(Vector3.forward, (Quaternion.Inverse(g.RobotFrame.rotation) * rotation) * Vector3.forward, Vector3.up),
            ScaraJointLab.IkSelection.PositiveElbow, ScaraJointLab.MotionMode.Joint, out _, out error)
            : g.Fanuc.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), Quaternion.Inverse(g.RobotFrame.rotation) * rotation,
                FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error);
        Require(ok, error); await Until(() => Rest(g)); await Frames(2);
    }
    private static bool Rest(RobotGripper g) => g.Scara != null ? !g.Scara.IsTrajectoryMoving && !g.Scara.Axis1.IsMoving && !g.Scara.Axis2.IsMoving && !g.Scara.Axis3.IsMoving && !g.Scara.Axis4.IsMoving
        : !g.Fanuc.IsTcpMoving && g.Fanuc.Joints.All(j => !j.IsMoving);
    private static async Task Frames(int count) { int frame = Time.frameCount; await Until(() => Time.frameCount - frame >= count); }
    private static async Task Until(Func<bool> condition) { var watch = Stopwatch.StartNew(); while (!condition()) { Require(watch.Elapsed.TotalSeconds < 15, "Verification wait timed out."); await Task.Delay(1); } }
    private static Dictionary<string, string> Hashes() => new[] { "Assets/01.Scenes/LineWatch.unity", "ProjectSettings/ProjectSettings.asset", "ProjectSettings/EditorBuildSettings.asset",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab", "Assets/03.Prefabs/GripTrialPart.prefab", "Assets/03.Prefabs/HandoffTrialPart.prefab",
        "Assets/04.ScriptableObject/ScaraCellHandoff.asset", "Assets/04.ScriptableObject/FanucCellHandoff.asset", "Assets/04.ScriptableObject/ScaraLabWaypoints.asset", "Assets/04.ScriptableObject/FanucLabWaypoints.asset" }
        .Concat(Directory.GetFiles("Assets/02.Scripts/LineWatch", "*.cs", SearchOption.AllDirectories)).Distinct().ToDictionary(p => p, Hash);
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW10"); File.WriteAllText("Logs/LW10/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
