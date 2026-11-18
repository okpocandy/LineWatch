using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace LineWatch
{
    [DefaultExecutionOrder(200)]
    [DisallowMultipleComponent]
    public sealed class CellProductTracker : MonoBehaviour
    {
        [SerializeField] private LineWatchCellLayout layout;
        [SerializeField] private GrippableItem initialProduct;
        [SerializeField] private string initialProductId = "P-000001";
        [SerializeField] private ProductDestination initialDestination = ProductDestination.A;

        private readonly ProductFlow flow = new ProductFlow();
        // Keep both original identities: a destroyed component does not prove that its
        // GameObject/physical body is gone. This is evidence, not another owner field.
        private readonly struct ProductBinding
        {
            public readonly GrippableItem Item;
            public readonly GameObject OriginalObject;
            public ProductBinding(GrippableItem item) { Item = item; OriginalObject = item.gameObject; }
        }
        private readonly Dictionary<string, ProductBinding> products = new Dictionary<string, ProductBinding>();
        private RobotGripper scaraGripper, fanucGripper;
        private CellCommandGate cellGate;
        private CellRunController cellController;
        private readonly List<QueueRecovery> queueRecoveries = new List<QueueRecovery>();
        private readonly HashSet<string> previousScenarioIds = new HashSet<string>();
        private readonly List<ScenarioSummary> scenarios = new List<ScenarioSummary>();
        public int ScenarioNumber { get; private set; } = 1;
        public ProductRecord[] CurrentScenarioSnapshot() => flow.Snapshot().Where(r => !previousScenarioIds.Contains(r.Id)).ToArray();
        public ScenarioSummary[] ScenarioSnapshot() => scenarios.ToArray();

        public sealed class ScenarioSummary
        {
            public int Number { get; }
            public string Reason { get; }
            public IReadOnlyList<ProductRecord> Products { get; }
            internal ScenarioSummary(int number, string reason, ProductRecord[] records)
            { Number = number; Reason = reason; Products = Array.AsReadOnly(records); }
        }

        public sealed class QueueRecovery
        {
            public ProductRecord Previous { get; }
            public ProductRecord Result { get; }
            public string Reason { get; }
            internal QueueRecovery(ProductRecord previous, ProductRecord result, string reason)
            { Previous = previous; Result = result; Reason = reason; }
        }
        public QueueRecovery[] QueueRecoverySnapshot() => queueRecoveries.ToArray();

        public bool IsFaulted { get; private set; }
        public string FaultReason { get; private set; }
        public string LastAcknowledgedFaultReason { get; private set; }
        public event Action<string> Faulted;
        public int Count => flow.Count;
        public string InputProductId => flow.InputProductId;
        public string ScaraProductId => flow.ScaraProductId;
        public string HandoffProductId => flow.HandoffProductId;
        public string FanucProductId => flow.FanucProductId;
        public IReadOnlyList<ProductRecord> Snapshot() => flow.Snapshot();
        public IReadOnlyList<string> WaitingSnapshot() => flow.WaitingSnapshot();
        public bool TryGet(string id, out ProductRecord record) => flow.TryGet(id, out record);
        public bool TryGetItem(string id, out GrippableItem item)
        {
            item = null;
            if (id == null || !products.TryGetValue(id, out var binding)) return false;
            item = binding.Item;
            return item != null;
        }

        private void Awake()
        {
            cellGate = new CellCommandGate(this);
            cellController = GetComponent<CellRunController>();
            if (layout == null) return;
            if (layout.Scara != null) scaraGripper = layout.Scara.GetComponent<RobotGripper>();
            if (layout.Fanuc != null) fanucGripper = layout.Fanuc.GetComponent<RobotGripper>();
        }

        private void Start()
        {
            if (initialProduct == null) return;
            if (!TryRegister(initialProduct, initialProductId, initialDestination, out var error) || !TryFeedNext(out _, out error)) Fault(error);
        }

        public bool TryRegister(GrippableItem item, string id, ProductDestination destination, out string error)
        {
            if (!cellGate.CanIssue(null, out error)) return false;
            if (!Ready(out error)) return false;
            if (item == null || !item.IsReady || item.Owner != null || item.IsTracked)
            { error = "활성 상태의 미보유·미등록 상품이 필요합니다."; return false; }
            if (item.transform.IsChildOf(layout.Scara.transform) || item.transform.IsChildOf(layout.Fanuc.transform))
            { error = "로봇 계층의 물체는 상품으로 등록할 수 없습니다."; return false; }
            if (!flow.TryEnqueue(id, destination, out _, out error)) return false;
            products.Add(id, new ProductBinding(item));
            item.BindProduct(this, id);
            return true;
        }

        public bool TryFeedNext(out ProductRecord record, out string error)
        {
            record = null;
            if (!cellGate.CanIssue(null, out error)) return false;
            if (!Ready(out error)) return false;
            if (flow.InputProductId != null) { error = "투입 지점 사용 중: 대기열을 유지합니다."; return false; }
            var waiting = flow.WaitingSnapshot();
            if (waiting.Count == 0) { error = "대기 중인 상품이 없습니다."; return false; }
            if (!TryGetItem(waiting[0], out var item) || !item.IsReady || item.Owner != null)
            { error = "선두 상품의 실제 물체를 확인하세요."; return false; }
            if (!At(item, layout.InputStop)) { error = "선두 상품이 투입 정지점에 정렬되지 않았습니다."; return false; }
            return flow.TryFeedNext(out record, out error);
        }

        // A loss receipt resolves one destroyed original, not a still-existing/disabled
        // item or a pending transport program. Fault acknowledgement remains separate.
        public bool CanConfirmDestroyedLoss(string id, ProductLocation expected, string reason, out string error)
        {
            error = null;
            if (!Application.isPlaying || !isActiveAndEnabled || !Wired || !IsFaulted)
            { error = "활성 상품 추적기가 고장을 유지하는 Play 상태에서만 손실을 확정합니다."; return false; }
            if (cellController == null || !cellGate.IsManagedBy(cellController))
            { error = "같은 셀의 운전 관리자 연결이 필요합니다."; return false; }
            if (!cellController.CanConfirmProductLoss(this, out error)) return false;
            if (string.IsNullOrWhiteSpace(reason)) { error = "손실 확정 사유가 필요합니다."; return false; }
            if (!flow.TryGet(id, out var record)) { error = "등록되지 않은 상품입니다."; return false; }
            if (record.Location != expected || Terminal(record.Location))
            { error = "현재 비종료 상품 상태와 손실 요청이 맞지 않습니다."; return false; }
            if (!products.TryGetValue(id, out var binding) || binding.Item != null || binding.OriginalObject != null)
            { error = "등록된 원래 GameObject의 삭제가 확인되어야 합니다. 컴포넌트 소실·비활성/탈락/위치 불일치는 미확정으로 유지합니다."; return false; }
            return true;
        }

        public bool TryConfirmDestroyedLoss(string id, ProductLocation expected, string reason, out string error)
        {
            if (!CanConfirmDestroyedLoss(id, expected, reason, out error)) return false;
            return flow.TryConfirmLoss(id, expected, reason, out _, out error);
        }

        public bool CanConfirmDestroyedLoss(ProductRecord expected, string reason, out string error)
        {
            if (expected == null || !flow.TryGet(expected.Id, out var current) || !ReferenceEquals(current, expected))
            { error = "선택 후 상품 기록이 바뀌었습니다. 대상을 비운 뒤 다시 선택하세요."; return false; }
            return CanConfirmDestroyedLoss(expected.Id, expected.Location, reason, out error);
        }

        public bool TryConfirmDestroyedLoss(ProductRecord expected, string reason, out string error)
        {
            if (!CanConfirmDestroyedLoss(expected, reason, out error)) return false;
            return flow.TryConfirmLoss(expected.Id, expected.Location, reason, out _, out error);
        }

        public bool CanReturnToQueue(ProductRecord expected, string reason, out string error)
        {
            error = null;
            if (!Application.isPlaying || !isActiveAndEnabled || !Wired || !IsFaulted || cellController == null)
            { error = "활성 상품 추적기와 Faulted 셀에서만 원래 상품을 대기로 복귀합니다."; return false; }
            if (!cellController.CanConfirmProductLoss(this, out error)) return false;
            if (string.IsNullOrWhiteSpace(reason)) { error = "대기 복귀 사유가 필요합니다."; return false; }
            if (expected == null || !flow.TryGet(expected.Id, out var current) || !ReferenceEquals(current, expected))
            { error = "선택 후 상품 기록이 바뀌었습니다. 대상을 다시 선택하세요."; return false; }
            if (current.Location != ProductLocation.Input && current.Location != ProductLocation.Scara &&
                current.Location != ProductLocation.Handoff && current.Location != ProductLocation.Fanuc)
            { error = "점유 중인 비종료 상품만 대기열 끝으로 복귀합니다."; return false; }
            if (!products.TryGetValue(current.Id, out var binding) || binding.Item == null || !binding.Item.IsReady ||
                binding.OriginalObject == null || binding.Item.gameObject != binding.OriginalObject || binding.Item.Owner != null ||
                binding.Item.ProductTracker != this || binding.Item.ProductId != current.Id)
            { error = "원래 상품의 활성·미보유·ID 연결을 확인하세요. 대체 물체나 실제 보유자를 강제 복구하지 않습니다."; return false; }
            return true;
        }

        public bool TryReturnToQueue(ProductRecord expected, string reason, out string error)
        {
            if (!CanReturnToQueue(expected, reason, out error)) return false;
            if (!flow.TryReturnToQueue(expected.Id, expected.Location, out var result, out error)) return false;
            queueRecoveries.Add(new QueueRecovery(expected, result, reason.Trim()));
            return true;
        }

        internal bool CanResetScenario(out string error)
        {
            error = null;
            if (IsFaulted || !TryCheckPhysicalState(out error))
            { if (error == null) error = "상품 고장을 먼저 명시 해제하세요."; return false; }
            if (CurrentScenarioSnapshot().Length == 0) { error = "이번 시나리오는 이미 비어 있습니다."; return false; }
            if (flow.Snapshot().Any(r => !Terminal(r.Location)) || flow.WaitingCount != 0 ||
                InputProductId != null || ScaraProductId != null || HandoffProductId != null || FanucProductId != null)
            { error = "모든 상품을 완료·제거·손실 확정한 뒤 시나리오를 초기화하세요."; return false; }
            return true;
        }

        internal void ResetScenarioFromCell(string reason)
        {
            scenarios.Add(new ScenarioSummary(ScenarioNumber, reason, CurrentScenarioSnapshot()));
            foreach (var record in flow.Snapshot()) previousScenarioIds.Add(record.Id);
            ScenarioNumber++;
        }

        public bool TryRemove(string id, ProductLocation expected, out string error)
        {
            if (!cellGate.CanIssue(null, out error)) return false;
            if (!Ready(out error)) return false;
            if (!TryGetItem(id, out var item) || !item.IsReady || item.Owner != null)
            { error = "미보유 상태의 실제 상품이 필요합니다."; return false; }
            if (!flow.TryRemove(id, expected, out _, out error)) return false;
            Destroy(item.gameObject);
            return true;
        }

        // Called only after the gripper has passed its physical readiness/alignment checks.
        internal bool TryCommitGrip(GrippableItem item, RobotGripper gripper, out string error)
        {
            if (!Ready(out error) || !Bound(item, out error)) return false;
            bool isScara = gripper == scaraGripper;
            if (!isScara && gripper != fanucGripper) { error = "이 셀의 로봇만 등록 상품을 집을 수 있습니다."; return false; }
            if (!At(item, isScara ? layout.InputStop : layout.HandoffPoint))
            { error = "상품이 지정된 집기 지점에 정렬되지 않았습니다."; return false; }
            return flow.TryTransfer(item.ProductId, isScara ? ProductLocation.Input : ProductLocation.Handoff,
                isScara ? ProductLocation.Scara : ProductLocation.Fanuc, out _, out error);
        }

        // Commit before ClearHeld; a rejected release preserves both the physical and logical owner.
        internal bool TryCommitRelease(GrippableItem item, RobotGripper gripper, out string error)
        {
            if (!Ready(out error) || !Bound(item, out error)) return false;
            bool isScara = gripper == scaraGripper;
            if ((!isScara && gripper != fanucGripper) || item.Owner != gripper || gripper.HeldItem != item)
            { error = "실제 상품 보유자가 셀 기록과 맞지 않습니다."; return false; }
            flow.TryGet(item.ProductId, out var record);
            var target = isScara ? layout.HandoffPoint : record.Destination == ProductDestination.A ? layout.OutputA : layout.OutputB;
            if (!At(item, target)) { error = "상품이 지정된 놓기 지점에 정렬되지 않았습니다."; return false; }
            return flow.TryTransfer(item.ProductId, isScara ? ProductLocation.Scara : ProductLocation.Fanuc,
                isScara ? ProductLocation.Handoff : record.Destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB,
                out _, out error);
        }

        internal void OwnerCleared(GrippableItem item)
        {
            if (flow.TryGet(item.ProductId, out var record) && (record.Location == ProductLocation.Scara || record.Location == ProductLocation.Fanuc))
                Fault("상품 " + record.Id + "의 실제 보유가 기록 갱신 없이 해제되었습니다.");
        }

        internal void ItemUnavailable(GrippableItem item)
        {
            if (flow.TryGet(item.ProductId, out var record) && !Terminal(record.Location)) Fault("상품 " + record.Id + "이 비활성화되거나 사라졌습니다.");
        }

        private void LateUpdate()
        {
            if (IsFaulted || products.Count == 0) return;
            if (!TryCheckPhysicalState(out var error)) Fault(error);
        }

        // Read the same physical evidence during normal monitoring and explicit recovery.
        // This query never clears a fault or changes the registry, owners or slot/FIFO data.
        internal bool TryCheckPhysicalState(out string error)
        {
            error = null;
            if (!isActiveAndEnabled || !Wired) { error = "셀의 상품 추적 연결이 유효하지 않습니다."; return false; }
            foreach (var pair in products)
            {
                flow.TryGet(pair.Key, out var record);
                var item = pair.Value.Item;
                if (Terminal(record.Location))
                {
                    if (record.Location == ProductLocation.Lost && (item != null || pair.Value.OriginalObject != null))
                    { error = "손실 확정 상품 " + record.Id + "의 원래 실물이 남아 있습니다."; return false; }
                    if (item != null && item.Owner != null) { error = "완료/제거 상품 " + record.Id + "이 실제로 보유되어 있습니다."; return false; }
                    continue;
                }
                if (item == null || !item.IsReady) { error = "상품 " + record.Id + "의 실제 물체가 유효하지 않습니다."; return false; }
                if (item.ProductTracker != this || item.ProductId != record.Id)
                { error = "상품 " + record.Id + "의 ID/실제 물체 대응이 다릅니다."; return false; }
                var owner = record.Location == ProductLocation.Scara ? scaraGripper : record.Location == ProductLocation.Fanuc ? fanucGripper : null;
                if (item.Owner != owner || owner != null && owner.HeldItem != item)
                { error = "상품 " + record.Id + "의 보유 기록과 실제 집게가 다릅니다."; return false; }
                var station = record.Location == ProductLocation.Input ? layout.InputStop : record.Location == ProductLocation.Handoff ? layout.HandoffPoint : null;
                if (station != null && !At(item, station)) { error = "상품 " + record.Id + "이 기록된 정지점에서 벗어났습니다."; return false; }
            }
            return true;
        }

        // The coordinator validates both robots and the saved program before this final
        // physical check. No event is emitted between the tracker and cell latch commits.
        internal bool TryAcknowledgeFault(CellRunController requester, out string error)
        {
            error = null;
            if (!Application.isPlaying || Time.timeScale <= 0f || requester == null || !requester.isActiveAndEnabled ||
                !cellGate.IsManagedBy(requester) || !requester.IsFaulted)
            { error = "활성 셀의 명시적 고장 해제 경로만 상품 추적 고장을 해제할 수 있습니다."; return false; }
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPaused) { error = "에디터 일시정지를 해제하세요."; return false; }
#endif
            if (!TryCheckPhysicalState(out error)) return false;
            if (IsFaulted)
            {
                LastAcknowledgedFaultReason = FaultReason;
                FaultReason = null;
                IsFaulted = false;
            }
            return true;
        }

        private bool Ready(out string error)
        {
            error = null;
            if (!Application.isPlaying || !isActiveAndEnabled || Time.timeScale <= 0) { error = "진행 중인 Play 상태가 필요합니다."; return false; }
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPaused) { error = "에디터 일시정지를 해제하세요."; return false; }
#endif
            if (IsFaulted) { error = FaultReason; return false; }
            if (!Wired)
            { error = "셀의 로봇/작업 지점 연결을 확인하세요."; return false; }
            return true;
        }

        private bool Bound(GrippableItem item, out string error)
        {
            error = null;
            if (item == null || item.ProductTracker != this || !TryGetItem(item.ProductId, out var known) || known != item)
            { error = "상품 ID와 실제 물체의 연결이 맞지 않습니다."; return false; }
            return true;
        }

        private static bool At(GrippableItem item, Transform point) => Vector3.Distance(item.Body.position, point.position) <= RobotGripper.PositionToleranceMetres &&
            RobotGripper.RotationError(item.Body.rotation, point.rotation) <= RobotGripper.RotationToleranceDegrees;
        private static bool Terminal(ProductLocation location) => location == ProductLocation.OutputA || location == ProductLocation.OutputB || location == ProductLocation.Removed || location == ProductLocation.Lost;
        private bool Wired => layout != null && layout.gameObject == gameObject && scaraGripper != null && fanucGripper != null &&
            scaraGripper.Scara == layout.Scara && fanucGripper.Fanuc == layout.Fanuc && layout.Scara.transform.IsChildOf(layout.transform) && layout.Fanuc.transform.IsChildOf(layout.transform) &&
            layout.InputStop != null && layout.HandoffPoint != null && layout.OutputA != null && layout.OutputB != null;
        private void Fault(string reason)
        {
            if (IsFaulted) return;
            IsFaulted = true; FaultReason = reason;
            if (scaraGripper != null) scaraGripper.Stop();
            if (fanucGripper != null) fanucGripper.Stop();
            Faulted?.Invoke(FaultReason);
        }
        private void OnDisable() { if (Application.isPlaying && flow.Count > 0) Fault("상품 추적 관리자가 비활성화되었습니다."); }
    }
}
