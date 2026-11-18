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

public static class VerifyDestroyedProductLoss
{
    public static async Task<string> Reproduce()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        await Until(() => t.Count == 1); Require(t.TryGetItem("P-000001", out var item), "Initial item missing.");
        UnityEngine.Object.Destroy(item.gameObject); await Until(() => t.IsFaulted && c.IsFaulted && !t.TryGetItem("P-000001", out _));
        bool removed = t.TryRemove("P-000001", ProductLocation.Input, out var error);
        var report = new { reproduced = !removed && t.InputProductId == "P-000001" && !t.TryGetItem("P-000001", out _),
            error, registeredProducts = t.Count, unconfirmedProducts = 1, cellFault = c.FaultReason, trackerFault = t.FaultReason };
        Write("destroyed-loss-before.json", report); return JsonConvert.SerializeObject(report);
    }
    public static async Task<string> ComponentOnlyBefore()
    {
        var l = Layout(); var t = l.GetComponent<CellProductTracker>(); var c = l.GetComponent<CellRunController>();
        await Until(() => t.Count == 1); Require(t.TryGetItem("P-000001", out var item), "Initial product missing.");
        var originalObject = item.gameObject; UnityEngine.Object.Destroy(item);
        await Until(() => !t.TryGetItem("P-000001", out _) && t.IsFaulted && c.IsFaulted);
        bool eligible = t.CanConfirmDestroyedLoss("P-000001", ProductLocation.Input, "컴포넌트만 삭제한 경우", out var error);
        var report = new { reproduced = eligible && originalObject != null, originalGameObjectStillExists = originalObject != null,
            lossWasEligible = eligible, error, receiptCreated = false };
        Write("destroyed-loss-component-before.json", report); return JsonConvert.SerializeObject(report);
    }

    public static async Task<string> Normal(int scenario)
    {
        Require(scenario >= 0 && scenario <= 5, "Unknown normal scenario.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        float scale = Time.timeScale; var hashes = Sources(); int rejected = 0, confirmations = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f); AddWaiting(l, "WAIT-C", 1.2f);
            var location = scenario == 1 ? ProductLocation.Waiting : scenario == 2 ? ProductLocation.Scara : scenario == 3 ? ProductLocation.Handoff : scenario == 4 ? ProductLocation.Fanuc : ProductLocation.Input;
            string id = scenario == 1 ? "WAIT-B" : "P-000001";
            var item = await PrepareLocation(l, location, id); var selected = scenario == 5 ? new[] { id, "WAIT-B" } : new[] { id };
            string others = OtherRecords(l, selected); var initialFifo = t.WaitingSnapshot().ToArray();
            UnityEngine.Object.Destroy(item.gameObject);
            if (scenario == 5) { Require(t.TryGetItem("WAIT-B", out var second), "Second loss fixture missing."); UnityEngine.Object.Destroy(second.gameObject); }
            await Until(() => t.IsFaulted && c.IsFaulted && selected.All(missing => !t.TryGetItem(missing, out _))); await Ticks(2);
            string cause = Faults(l), before = Full(l); const string reason = "검증 중 원래 상품 오브젝트 삭제 확인";
            for (int i = 0; i < 100; i++) Require(t.CanConfirmDestroyedLoss(id, location, reason, out var e), e);
            Require(Full(l) == before, "Loss eligibility query changed records, faults or physical state.");
            Require(!t.TryRemove(id, location, out _) && Full(l) == before, "Destroyed original was treated as normal removal."); rejected++;
            Confirm(id, location, reason);
            if (scenario == 5)
            {
                before = Full(l);
                Require(!c.CanExecute(CellRunCommand.Reset, out _) && !c.TryExecute(CellRunCommand.Reset, out _) && Full(l) == before,
                    "One loss receipt normalized a second unresolved product or cleared faults."); rejected++;
                Require(t.TryGet("WAIT-B", out var unresolved) && unresolved.Location == ProductLocation.Waiting && t.WaitingSnapshot().Contains("WAIT-B"), "Other unresolved record/FIFO was silently released.");
                Confirm("WAIT-B", ProductLocation.Waiting, "두 번째 원래 상품 삭제 확인");
            }
            Require(t.Count == 4 && OtherRecords(l, selected) == others && t.WaitingSnapshot().SequenceEqual(initialFifo.Where(queued => !selected.Contains(queued))), "Loss changed survivors/total/FIFO order.");
            Require(Faults(l) == cause && c.IsFaulted && t.IsFaulted, "Loss confirmation acknowledged a fault automatically.");
            var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show(); w.RefreshCell();
            var text = w.rootVisualElement.Q<Label>("cell-run-products").text;
            Require(text.Contains("명시 손실 확정 " + confirmations) && text.Contains(id) && text.Contains(reason), "Explicit loss receipt/reason missing from read-only UI.");
            before = Full(l); await Ticks(3); Require(Full(l) == before, "Loss receipt automatically advanced the cell or reset history.");
            string data = Records(l), motion = Motion(l); string cellCause = c.FaultReason, trackerCause = t.FaultReason;
            Require(c.TryExecute(CellRunCommand.Reset, out var error), error);
            Require(c.State == CellRunState.Stopped && !t.IsFaulted && Records(l) == data && Motion(l) == motion &&
                c.LastAcknowledgedFaultReason == cellCause && t.LastAcknowledgedFaultReason == trackerCause,
                "Separate acknowledgement cleared loss receipts, dispatched or lost fault history.");
            var candidate = UnityEngine.Object.Instantiate(l.ProductPrefab); candidate.Body.position = l.InputStop.position + Vector3.right * 1.6f;
            Require(!t.TryRegister(candidate, id, ProductDestination.B, out error) && error.Contains("이미 등록") && !candidate.IsTracked && t.Count == 4,
                "Lost ID was reused after explicit fault acknowledgement."); rejected++; UnityEngine.Object.Destroy(candidate.gameObject);
            var records = t.Snapshot(); int lost = records.Count(r => r.Location == ProductLocation.Lost);
            int residual = records.Count(r => !Terminal(r.Location));
            Require(lost == confirmations && records.Where(r => !Terminal(r.Location)).All(r => t.TryGetItem(r.Id, out var survivor) && survivor.IsReady && survivor.Owner == null) &&
                t.Count == lost + residual && !t.IsFaulted && !c.IsFaulted, "Loss-inclusive conservation is not physically reconciled.");
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, explicitLossConfirmations = confirmations,
                eligibilityQueriesPreserved = 100, rejectedRequestsPreserved = rejected, registeredProducts = t.Count,
                completedProducts = 0, removedProducts = 0, confirmedLost = lost, physicallyConfirmedResidual = residual, unconfirmedProducts = 0,
                faultsRemainedAfterConfirmation = true, separateFaultAcknowledgement = true, lostIdsCannotBeReused = true,
                motionCommandsAtConfirmation = 0, sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, hashes };
            Write("destroyed-loss-normal-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.explicitLossConfirmations, report.registeredProducts, report.confirmedLost, report.physicallyConfirmedResidual, report.unconfirmedProducts });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
        void Confirm(string target, ProductLocation expected, string explanation)
        {
            string motion = Motion(l), faults = Faults(l); int count = t.Count;
            Require(t.TryGet(target, out var original), "Loss target missing.");
            Require(t.TryConfirmDestroyedLoss(target, expected, explanation, out var error), error); confirmations++;
            Require(t.TryGet(target, out var receipt) && receipt.Location == ProductLocation.Lost && receipt.LostFrom == expected &&
                receipt.LossReason == explanation && receipt.Id == original.Id && receipt.Destination == original.Destination &&
                t.Count == count && Motion(l) == motion && Faults(l) == faults && !t.TryGetItem(target, out _) &&
                new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId }.All(slot => slot != target),
                "Loss receipt erased identity/cause or changed physical ownership/motor commands.");
            string after = Full(l);
            Require(!t.TryConfirmDestroyedLoss(target, expected, "덮어쓰기 금지", out _) &&
                !t.TryConfirmDestroyedLoss(target, ProductLocation.Lost, "덮어쓰기 금지", out _) && Full(l) == after,
                "Repeated confirmation overwrote the first loss receipt."); rejected += 2;
        }
    }

    public static async Task<string> Blocked(int scenario)
    {
        Require(scenario >= 0 && scenario <= 7, "Unknown blocked scenario.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var q = sg.GetComponent<RobotWaypointSequence>();
        float scale = Time.timeScale; bool paused = EditorApplication.isPaused; var hashes = Sources(); int rejected = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f); AddWaiting(l, "WAIT-C", 1.2f);
            string id = scenario == 3 || scenario == 4 ? "WAIT-B" : "P-000001";
            var expected = scenario == 1 ? ProductLocation.Scara : scenario == 3 || scenario == 4 ? ProductLocation.Waiting : ProductLocation.Input;
            var item = await PrepareLocation(l, scenario == 1 || scenario == 4 ? ProductLocation.Scara : ProductLocation.Input, "P-000001");
            if (scenario == 0)
            { item.gameObject.SetActive(false); await Until(() => t.IsFaulted && c.IsFaulted); Reject(id, expected, "비활성 원본"); item.gameObject.SetActive(true); Reject(id, expected, "재활성 원본"); }
            else if (scenario == 1)
            { item.gameObject.SetActive(false); item.gameObject.SetActive(true); await Until(() => t.IsFaulted && c.IsFaulted); Reject(id, expected, "실물이 남은 보유 불일치"); }
            else if (scenario == 7)
            {
                var originalObject = item.gameObject; var body = item.Body; var position = body.position;
                UnityEngine.Object.Destroy(item); await Until(() => !t.TryGetItem(id, out _) && t.IsFaulted && c.IsFaulted);
                Require(originalObject != null && originalObject.activeInHierarchy && body != null, "Fixture must retain the original physical GameObject.");
                Reject(id, expected, "컴포넌트만 삭제된 실물");
                Require(originalObject != null && body != null && Vector3.Distance(body.position, position) < .000001f, "Component loss confirmation removed or moved the remaining physical object.");
            }
            else if (scenario == 6)
            {
                item = await PrepareLocation(l, ProductLocation.Fanuc, "P-000001"); var fg = l.Fanuc.GetComponent<RobotGripper>();
                await Move(fg, item, l.OutputA); Require(fg.TryRelease(out var e), e); Require(t.TryRemove("WAIT-A", ProductLocation.Waiting, out e), e);
                UnityEngine.Object.Destroy(item.gameObject); await Until(() => !t.TryGetItem("P-000001", out _)); t.enabled = false; t.enabled = true;
                Reject("P-000001", ProductLocation.OutputA, "완료 이력 변경 금지"); Reject("WAIT-A", ProductLocation.Removed, "명시 제거 이력 변경 금지");
            }
            else
            {
                if (scenario == 3) Require(q.TryStart(out var e), e);
                Require(t.TryGetItem(id, out var target), "Selected loss target missing."); UnityEngine.Object.Destroy(target.gameObject);
                await Until(() => t.IsFaulted && c.IsFaulted && !t.TryGetItem(id, out _)); await Ticks(2);
                if (scenario == 2)
                {
                    Reject(id, expected, null); Reject(id, expected, " "); Reject(id, ProductLocation.Waiting, "오래된 상태");
                    Reject(id, (ProductLocation)99, "잘못된 상태"); Reject(null, expected, "ID 없음"); Reject("UNKNOWN", expected, "미등록 ID");
                }
                else if (scenario == 5)
                {
                    c.enabled = false; Reject(id, expected, "관리자 비활성"); c.enabled = true;
                    t.enabled = false; Reject(id, expected, "추적기 비활성"); t.enabled = true;
                    Time.timeScale = 0; Reject(id, expected, "시간 0"); Time.timeScale = 1;
                    EditorApplication.isPaused = true; Reject(id, expected, "Editor Pause"); EditorApplication.isPaused = false;
                    Require(t.CanConfirmDestroyedLoss(id, expected, "복원 후 명시 요청 필요", out _), "Restored preconditions should allow only an explicit future request.");
                }
                else Reject(id, expected, scenario == 3 ? "미완료 저장 경로" : "다른 실제 상품 보유");
            }
            Require(t.Snapshot().All(r => r.Location != ProductLocation.Lost) && t.Count == 4 && t.IsFaulted && c.IsFaulted,
                "Rejected confirmation cleared a fault, normalized an unknown or erased registration history.");
            if (scenario == 1) Require(item.Owner == null && sg.HeldItem == null && t.ScaraProductId == "P-000001", "Existing physical item became a normal residual/holding.");
            if (scenario == 3) Require(q.State == State.Paused && !c.HasControlledSequence, "Pending program was abandoned by loss confirmation.");
            if (scenario == 4) Require(sg.HeldItem == item && item.Owner == sg && t.ScaraProductId == "P-000001", "Other physical holding was released.");
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, rejectedRequestsPreserved = rejected,
                confirmedLost = 0, faultsRemainLatched = true, noPartialReceiptOrSlotChange = true, componentOnlyDeletionBlocked = scenario == 7, sourcesPreserved = hashes.Count,
                directButtonClicks = 0, callbacksInjected = 0, hashes };
            Write("destroyed-loss-blocked-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.rejectedRequestsPreserved, report.confirmedLost, report.faultsRemainLatched });
        }
        finally { EditorApplication.isPaused = paused; Time.timeScale = scale; c.enabled = true; t.enabled = true; c.TryExecute(CellRunCommand.Stop, out _); }
        void Reject(string target, ProductLocation location, string explanation)
        { string before = Full(l); Require(!t.TryConfirmDestroyedLoss(target, location, explanation, out var e) && !string.IsNullOrWhiteSpace(e) && Full(l) == before, "Rejected loss confirmation changed evidence: " + explanation); rejected++; }
    }

    private static async Task<GrippableItem> PrepareLocation(LineWatchCellLayout l, ProductLocation location, string id)
    {
        var t = l.GetComponent<CellProductTracker>(); Require(t.TryGetItem(id, out var item), "Original product missing.");
        if (location == ProductLocation.Input || location == ProductLocation.Waiting) return item;
        var sg = l.Scara.GetComponent<RobotGripper>(); await Move(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
        if (location == ProductLocation.Scara) return item;
        await Move(sg, item, l.HandoffPoint); Require(sg.TryRelease(out e), e);
        if (location == ProductLocation.Handoff) return item;
        var retreat = l.ScaraHandoffWaypoints.Retreat; Require(l.Scara.TryMoveTcp(retreat.PositionMetres, Yaw(retreat.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out e), e); await Until(() => sg.IsStopped);
        var fg = l.Fanuc.GetComponent<RobotGripper>(); await Move(fg, item, l.HandoffPoint); Require(fg.TryGrip(item, out e), e); return item;
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
    private static void AddWaiting(LineWatchCellLayout l, string id, float offset)
    { var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset; Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e); }
    private static bool Terminal(ProductLocation p) => p == ProductLocation.OutputA || p == ProductLocation.OutputB || p == ProductLocation.Removed || p == ProductLocation.Lost;
    private static string OtherRecords(LineWatchCellLayout l, string[] exclude)
    { var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(t.Snapshot().Where(r => !exclude.Contains(r.Id)).Select(r => new { record = r,
        instance = t.TryGetItem(r.Id, out var item) ? item.GetInstanceID() : 0, owner = item != null && item.Owner != null ? item.Owner.GetInstanceID() : 0,
        ready = item != null && item.IsReady, active = item != null && item.gameObject.activeSelf,
        position = item != null ? new[] { item.Body.position.x, item.Body.position.y, item.Body.position.z } : null,
        rotation = item != null ? new[] { item.Body.rotation.x, item.Body.rotation.y, item.Body.rotation.z, item.Body.rotation.w } : null })); }
    private static string Records(LineWatchCellLayout l)
    { var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(), slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId }, t.Count }); }
    private static string Motion(LineWatchCellLayout l)
    { var c = l.GetComponent<CellRunController>(); return JsonConvert.SerializeObject(new { c.HasControlledSequence, controlled = c.ControlledSequence != null ? c.ControlledSequence.GetInstanceID() : 0,
        robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() }.Select(g => {
            var q = g.GetComponent<RobotWaypointSequence>(); return new { q.State, q.StepIndex, q.IsPickPlace, q.GripCount, q.ReleaseCount, q.LastError, q.LastAcknowledgedFaultReason,
                version = g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion,
                current = g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition } : g.Fanuc.Joints.Select(a => a.CurrentPosition).ToArray(),
                targets = g.Scara != null ? new[] { g.Scara.Axis1.TargetPosition, g.Scara.Axis2.TargetPosition, g.Scara.Axis3.TargetPosition, g.Scara.Axis4.TargetPosition } : g.Fanuc.Joints.Select(a => a.TargetPosition).ToArray(),
                held = g.HeldItem != null ? g.HeldItem.GetInstanceID() : 0 }; }) }); }
    private static string Faults(LineWatchCellLayout l)
    { var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.LastAcknowledgedFaultReason, t.IsFaulted, trackerCause = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason }); }
    private static string Full(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { records = Records(l), physical = OtherRecords(l, new string[0]), motion = Motion(l), faults = Faults(l) });
    private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
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
    private static async Task Ticks(int n) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= n * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
