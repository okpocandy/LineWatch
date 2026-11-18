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

public static class VerifyProgramCancellation
{
    public static async Task<string> Run(int scenario)
    {
        Require(scenario >= 0 && scenario <= 7, "Unknown scenario.");
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use a fresh unpaused Play session.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch.");
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = scenario == 1 || scenario == 5 ? l.Fanuc.GetComponent<RobotGripper>() : l.Scara.GetComponent<RobotGripper>();
        var q = g.GetComponent<RobotWaypointSequence>();
        var other = (g.Scara != null ? l.Fanuc.gameObject : l.Scara.gameObject).GetComponent<RobotWaypointSequence>();
        var original = q.Waypoints; var copy = UnityEngine.Object.Instantiate(original);
        var hashes = Sources(); float scale = Time.timeScale; int rejected = 0, cancelled = 0;
        const string reason = "검증: 저장 경로를 명시 종료하고 이력과 실제 보유를 보존";
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1);
            AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
            Require(t.TryGetItem("P-000001", out var input), "Input missing.");
            Reject(q, reason, "healthy idle");
            SetWaypoints(q, copy);
            GrippableItem pathItem = null;
            if (scenario == 4)
            {
                await MoveToInput(g, input, l.InputStop);
                Require(g.TryGrip(input, out var e), e);
            }
            if (scenario == 3 || scenario == 5)
            {
                var lab = g.GetComponent<RobotGripperLab>(); Require(lab.TryPrepare(out var e), e); pathItem = lab.TrialItem;
                if (scenario == 3) Require(t.TryRegister(pathItem, "PATH-ITEM", ProductDestination.B, out e), e);
                Time.timeScale = .1f; Require(q.TryStartPickPlace(g, pathItem, out e), e);
                if (scenario == 5) { Time.timeScale = 5; await Until(() => q.GripCount == 1 && q.StepIndex >= 2 || c.IsFaulted); Time.timeScale = .1f; }
            }
            else { Time.timeScale = .1f; Require(q.TryStart(out var e), e); }
            Reject(q, reason, "healthy running");
            if (scenario == 1 || scenario == 5)
            {
                SetRetreat(copy, original.Retreat.PositionMetres + Vector3.right * .01f);
                await Until(() => q.State == State.Faulted && c.IsFaulted);
            }
            else
            {
                q.Pause(); Require(q.State == State.Paused, "Expected paused program.");
                Reject(q, reason, "healthy paused");
                if (scenario == 2 || scenario == 4) Require(c.TryTakeControl(q, out var e), e);
                if (scenario == 6) { Require(other.TryStart(out var e), e); other.Pause(); }
            }
            string lossId = scenario == 3 ? "PATH-ITEM" : "P-000001";
            if (scenario <= 3 || scenario >= 6)
            {
                if (scenario == 7) UnityEngine.Object.Destroy(input);
                else UnityEngine.Object.Destroy((scenario == 3 ? pathItem : input).gameObject);
                await Until(() => c.IsFaulted && t.IsFaulted && !t.TryGetItem(lossId, out _));
            }
            else { t.enabled = false; t.enabled = true; }
            Time.timeScale = 1; await Ticks(2);
            Require(c.IsFaulted && t.IsFaulted && (q.State == State.Paused || q.State == State.Faulted), "Fault fixture failed.");
            string before = Full(l); for (int i = 0; i < 100; i++) Require(c.CanCancelProgram(q, reason, out var e), e);
            Require(Full(l) == before, "Eligibility query changed state.");
            Reject(null, reason, "no target"); Reject(q, null, "no reason"); Reject(q, " ", "blank reason");
            var foreign = new GameObject("ForeignCancellationTarget");
            try { Reject(foreign.AddComponent<RobotWaypointSequence>(), reason, "foreign target"); }
            finally { UnityEngine.Object.Destroy(foreign); }
            if (scenario != 6) Reject(other, reason, "idle other target");
            Time.timeScale = 0; Reject(q, reason, "time stopped"); Time.timeScale = 1;
            EditorApplication.isPaused = true; Reject(q, reason, "Editor paused"); EditorApplication.isPaused = false;
            c.enabled = false; Reject(q, reason, "controller disabled"); c.enabled = true;
            t.enabled = false; Reject(q, reason, "tracker disabled"); t.enabled = true;
            q.enabled = false; Reject(q, reason, "sequence disabled"); q.enabled = true;
            Require(q.CancellationSnapshot().Length == 0 && c.IsFaulted && t.IsFaulted, "Readiness repair implicitly cancelled or cleared faults.");
            string productData = Products(l), faults = Faults(l), motion = Motion(l), saved = Saved(q), otherState = Path(other);
            State oldState = q.State; string oldCause = q.LastError; var held = g.HeldItem;
            Require(c.TryCancelProgram(q, reason, out var error), error); cancelled++;
            Require(q.State == State.Cancelled && !q.HasPendingProgram && !q.IsBusy && Saved(q) == saved && Products(l) == productData &&
                Faults(l) == faults && Motion(l) == motion && Path(other) == otherState && g.HeldItem == held,
                "Cancellation changed product/FIFO/holding/actions/motion/faults or the other path.");
            if (scenario == 2 || scenario == 4) Require(!c.HasControlledSequence && c.ControlledSequence == null, "Cancelled adopted path retained command ownership.");
            var receipt = q.CancellationSnapshot().Single();
            Require(receipt.Reason == reason && receipt.PreviousState == oldState && receipt.StepIndex == q.StepIndex &&
                receipt.GripCount == q.GripCount && receipt.ReleaseCount == q.ReleaseCount && receipt.FaultReason == oldCause &&
                receipt.Targets.Count == 4 && receipt.Targets[3].position == original.Retreat.PositionMetres,
                "Cancellation receipt lost saved-program evidence.");
            if (scenario == 3) Require(receipt.ProductId == "PATH-ITEM", "Destroyed cycle item ID was lost.");
            string archive = Receipt(receipt);
            var exposed = q.CancellationSnapshot(); exposed[0] = null;
            Require(Receipt(q.CancellationSnapshot().Single()) == archive, "Receipt array exposed writable history.");
            var targets = receipt.Targets as IList<Pose>; bool readOnly = false;
            try { targets[0] = default; } catch (NotSupportedException) { readOnly = true; }
            Require(readOnly, "Receipt target evidence was writable.");
            Reject(q, reason, "duplicate cancellation"); RejectResume("faulted cancellation");
            before = Full(l); Require(!c.TryExecute(CellRunCommand.Start, out _) && Full(l) == before, "Cancellation started/acknowledged the cell."); rejected++;
            await Ticks(3); Require(Full(l) == before, "Cancellation automatically progressed in later frames.");
            var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show(); w.RefreshCell();
            Require(w.rootVisualElement.Query<Label>().ToList().Any(label => label.text != null && label.text.Contains("명시 종료") && label.text.Contains(reason)), "Read-only cancellation receipt missing from common UI.");
            if (scenario == 7)
            {
                before = Full(l);
                Require(!t.TryConfirmDestroyedLoss(lossId, ProductLocation.Input, reason, out _) && !c.TryExecute(CellRunCommand.Reset, out _) && Full(l) == before,
                    "Cancelling a path normalized a remaining physical object whose tracking component disappeared."); rejected += 2;
                c.enabled = false; c.enabled = true; t.enabled = false; t.enabled = true; await Ticks(2);
                Require(Full(l) == before && c.IsFaulted && t.IsFaulted && q.State == State.Cancelled,
                    "Reactivation normalized unresolved physical evidence or erased cancellation history.");
            }
            else
            {
                if (scenario <= 3 || scenario == 6)
                {
                    var location = scenario == 3 ? ProductLocation.Waiting : ProductLocation.Input;
                    if (scenario == 6)
                    {
                        before = Full(l); Require(!t.TryConfirmDestroyedLoss(lossId, location, reason, out _) && Full(l) == before, "Cancelling one path erased another pending path."); rejected++;
                        Require(c.TryCancelProgram(other, "두 번째 미완료 경로도 별도 종료", out error), error); cancelled++;
                    }
                    Require(t.TryConfirmDestroyedLoss(lossId, location, "경로 종료 후 별도 원본 삭제 확정", out error), error);
                    Require(c.IsFaulted && t.IsFaulted && q.State == State.Cancelled, "Separate loss acknowledgement cleared faults or revived a path.");
                }
                Require(c.TryExecute(CellRunCommand.Reset, out error), error);
                Require(c.State == CellRunState.Stopped && !t.IsFaulted && q.State == State.Cancelled && g.HeldItem == held, "Separate fault acknowledgement resumed or lost holding.");
                RejectResume("acknowledged cancellation");
                if (scenario == 5) Require(g.TryRelease(out error), error);
                SetWaypoints(q, original); Require(q.TryStart(out error), error); q.Pause();
                Require(q.State == State.Paused && Receipt(q.CancellationSnapshot().Single()) == archive, "New explicit program start erased cancellation evidence.");
            }
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, cancelledPrograms = cancelled,
                eligibilityQueriesPreserved = 100, rejectedRequestsPreserved = rejected, receiptSurvivedNewProgram = scenario != 7, unresolvedPhysicalEvidencePreserved = scenario == 7,
                productHoldingFifoActionsFaultsPreserved = true, motionCommandsAtCancellation = 0, sourcesPreserved = hashes.Count,
                directButtonClicks = 0, callbacksInjected = 0, uiInputsAssigned = 0, hashes };
            Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/path-cancellation-" + scenario + ".json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.cancelledPrograms, report.rejectedRequestsPreserved });
        }
        finally { EditorApplication.isPaused = false; Time.timeScale = scale; c.enabled = true; t.enabled = true; q.enabled = true; c.TryExecute(CellRunCommand.Stop, out _); SetWaypoints(q, original); UnityEngine.Object.Destroy(copy); }
        void Reject(RobotWaypointSequence target, string why, string name)
        { string before = Full(l); Require(!c.CanCancelProgram(target, why, out _) && !c.TryCancelProgram(target, why, out var e) && !string.IsNullOrEmpty(e) && Full(l) == before, "Rejected request mutated state: " + name); rejected++; }
        void RejectResume(string name)
        { string before = Full(l); Require(!q.TryResume(out _) && Full(l) == before, "Cancelled program resumed: " + name); rejected++; }
    }

    private static async Task MoveToInput(RobotGripper g, GrippableItem item, Transform station)
    {
        var point = item.GetGripPoint(RobotWaypointSet.RobotType.Scara);
        var rotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = station.TransformPoint(point.localPosition) - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        var local = Quaternion.Inverse(g.RobotFrame.rotation) * rotation;
        float yaw = Vector3.SignedAngle(Vector3.forward, local * Vector3.forward, Vector3.up);
        Require(g.Scara.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), yaw, ScaraJointLab.IkSelection.PositiveElbow, out _, out var e), e);
        await Until(() => g.IsStopped); await Ticks(2);
    }
    private static void AddWaiting(LineWatchCellLayout l, string id, float offset)
    { var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset; Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e); }
    private static void SetWaypoints(RobotWaypointSequence q, RobotWaypointSet set)
    { var so = new SerializedObject(q); so.FindProperty("waypoints").objectReferenceValue = set; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetRetreat(RobotWaypointSet set, Vector3 p)
    { var so = new SerializedObject(set); so.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = p; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static string Saved(RobotWaypointSequence q) => JsonConvert.SerializeObject(new { q.StepIndex, q.LastArrivedStepIndex, arrival = P(q.LastArrivalPose), release = P(q.LastReleasePose), q.IsPickPlace, q.GripCount, q.ReleaseCount, q.LastError, q.LastAcknowledgedFaultReason, item = Ref(q.CycleItem) });
    private static string Receipt(RobotWaypointSequence.ProgramRecord r) => JsonConvert.SerializeObject(new { r.Reason, r.ProgramName, r.ProductId, r.PreviousState, r.StepIndex, r.LastArrivedStepIndex, arrival = P(r.LastArrivalPose), release = P(r.LastReleasePose), r.IsPickPlace, r.GripCount, r.ReleaseCount, r.FaultReason, targets = r.Targets.Select(P) });
    private static string Path(RobotWaypointSequence q) => JsonConvert.SerializeObject(new { q.State, saved = Saved(q), receipts = q.CancellationSnapshot().Select(Receipt) });
    private static string Products(LineWatchCellLayout l)
    {
        var t = l.GetComponent<CellProductTracker>();
        return JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(), slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            physical = UnityEngine.Object.FindObjectsByType<GrippableItem>(FindObjectsInactive.Include, FindObjectsSortMode.InstanceID).Select(i => new { id = Ref(i), i.ProductId, owner = Ref(i.Owner), active = i.gameObject.activeInHierarchy, position = V(i.Body.position), rotation = Q(i.Body.rotation) }) });
    }
    private static string Faults(LineWatchCellLayout l)
    { var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.LastAcknowledgedFaultReason, t.IsFaulted, trackerReason = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason }); }
    private static string Motion(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { scara = l.Scara.CommandVersion, fanuc = l.Fanuc.CommandVersion,
        positions = new[] { l.Scara.Axis1.CurrentPosition, l.Scara.Axis2.CurrentPosition, l.Scara.Axis3.CurrentPosition, l.Scara.Axis4.CurrentPosition }.Concat(l.Fanuc.Joints.Select(j => j.CurrentPosition)),
        targets = new[] { l.Scara.Axis1.TargetPosition, l.Scara.Axis2.TargetPosition, l.Scara.Axis3.TargetPosition, l.Scara.Axis4.TargetPosition }.Concat(l.Fanuc.Joints.Select(j => j.TargetPosition)),
        scaraHeld = Ref(l.Scara.GetComponent<RobotGripper>().HeldItem), fanucHeld = Ref(l.Fanuc.GetComponent<RobotGripper>().HeldItem) });
    private static string Full(LineWatchCellLayout l)
    { var c = l.GetComponent<CellRunController>(); return JsonConvert.SerializeObject(new { products = Products(l), faults = Faults(l), motion = Motion(l), c.HasControlledSequence, controlled = Ref(c.ControlledSequence), paths = new[] { Path(l.Scara.GetComponent<RobotWaypointSequence>()), Path(l.Fanuc.GetComponent<RobotWaypointSequence>()) } }); }
    private static int Ref(UnityEngine.Object o) => ReferenceEquals(o, null) ? 0 : o.GetInstanceID();
    private static float[] V(Vector3 v) => new[] { v.x, v.y, v.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static object P(Pose p) => new { position = V(p.position), rotation = Q(p.rotation) };
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }.SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", ""); }
    private static void CheckSources(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification modified source/assets.");
    private static async Task Ticks(int count) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= count * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 20, "Timed out waiting for simulation progress."); await Task.Delay(1); } }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
