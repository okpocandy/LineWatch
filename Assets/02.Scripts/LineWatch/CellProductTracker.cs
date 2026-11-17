using System.Collections.Generic;
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
        private readonly Dictionary<string, GrippableItem> products = new Dictionary<string, GrippableItem>();
        private RobotGripper scaraGripper, fanucGripper;

        public bool IsFaulted { get; private set; }
        public string FaultReason { get; private set; }
        public int Count => flow.Count;
        public string InputProductId => flow.InputProductId;
        public string ScaraProductId => flow.ScaraProductId;
        public string HandoffProductId => flow.HandoffProductId;
        public string FanucProductId => flow.FanucProductId;
        public IReadOnlyList<ProductRecord> Snapshot() => flow.Snapshot();
        public IReadOnlyList<string> WaitingSnapshot() => flow.WaitingSnapshot();
        public bool TryGet(string id, out ProductRecord record) => flow.TryGet(id, out record);
        public bool TryGetItem(string id, out GrippableItem item)
        { item = null; return id != null && products.TryGetValue(id, out item) && item != null; }

        private void Awake()
        {
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
            if (!Ready(out error)) return false;
            if (item == null || !item.IsReady || item.Owner != null || item.IsTracked)
            { error = "활성 상태의 미보유·미등록 상품이 필요합니다."; return false; }
            if (item.transform.IsChildOf(layout.Scara.transform) || item.transform.IsChildOf(layout.Fanuc.transform))
            { error = "로봇 계층의 물체는 상품으로 등록할 수 없습니다."; return false; }
            if (!flow.TryEnqueue(id, destination, out _, out error)) return false;
            products.Add(id, item);
            item.BindProduct(this, id);
            return true;
        }

        public bool TryFeedNext(out ProductRecord record, out string error)
        {
            record = null;
            if (!Ready(out error)) return false;
            if (flow.InputProductId != null) { error = "투입 지점 사용 중: 대기열을 유지합니다."; return false; }
            var waiting = flow.WaitingSnapshot();
            if (waiting.Count == 0) { error = "대기 중인 상품이 없습니다."; return false; }
            if (!TryGetItem(waiting[0], out var item) || !item.IsReady || item.Owner != null)
            { error = "선두 상품의 실제 물체를 확인하세요."; return false; }
            if (!At(item, layout.InputStop)) { error = "선두 상품이 투입 정지점에 정렬되지 않았습니다."; return false; }
            return flow.TryFeedNext(out record, out error);
        }

        public bool TryRemove(string id, ProductLocation expected, out string error)
        {
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
            if (IsFaulted) return;
            if (products.Count > 0 && !Wired) { Fault("셀의 추적 연결이 바뀌었습니다."); return; }
            foreach (var pair in products)
            {
                flow.TryGet(pair.Key, out var record);
                if (Terminal(record.Location)) continue;
                var item = pair.Value;
                if (item == null || !item.IsReady) { Fault("상품 " + record.Id + "의 실제 물체가 유효하지 않습니다."); return; }
                var owner = record.Location == ProductLocation.Scara ? scaraGripper : record.Location == ProductLocation.Fanuc ? fanucGripper : null;
                if (item.Owner != owner || owner != null && owner.HeldItem != item)
                { Fault("상품 " + record.Id + "의 보유 기록과 실제 집게가 다릅니다."); return; }
                var station = record.Location == ProductLocation.Input ? layout.InputStop : record.Location == ProductLocation.Handoff ? layout.HandoffPoint : null;
                if (station != null && !At(item, station)) { Fault("상품 " + record.Id + "이 기록된 정지점에서 벗어났습니다."); return; }
            }
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
        private static bool Terminal(ProductLocation location) => location == ProductLocation.OutputA || location == ProductLocation.OutputB || location == ProductLocation.Removed;
        private bool Wired => layout != null && layout.gameObject == gameObject && scaraGripper != null && fanucGripper != null &&
            scaraGripper.Scara == layout.Scara && fanucGripper.Fanuc == layout.Fanuc && layout.Scara.transform.IsChildOf(layout.transform) && layout.Fanuc.transform.IsChildOf(layout.transform) &&
            layout.InputStop != null && layout.HandoffPoint != null && layout.OutputA != null && layout.OutputB != null;
        private void Fault(string reason)
        {
            if (IsFaulted) return;
            IsFaulted = true; FaultReason = reason;
            if (scaraGripper != null) scaraGripper.Stop();
            if (fanucGripper != null) fanucGripper.Stop();
        }
        private void OnDisable() { if (Application.isPlaying && flow.Count > 0) Fault("상품 추적 관리자가 비활성화되었습니다."); }
    }
}
