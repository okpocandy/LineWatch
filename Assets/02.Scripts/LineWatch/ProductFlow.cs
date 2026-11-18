using System.Collections.Generic;
using System.Linq;

namespace LineWatch
{
    // One mutation path owns the logical registry, FIFO and slot occupancy together.
    // Physical grip and scene objects are connected in a later step.
    public sealed class ProductFlow
    {
        private readonly ProductRegistry registry = new ProductRegistry();
        private Queue<string> waiting = new Queue<string>();
        private readonly Dictionary<ProductLocation, string> slots = new Dictionary<ProductLocation, string>
        {
            { ProductLocation.Input, null }, { ProductLocation.Scara, null },
            { ProductLocation.Handoff, null }, { ProductLocation.Fanuc, null }
        };

        public int Count => registry.Count;
        public int WaitingCount => waiting.Count;
        public string InputProductId => slots[ProductLocation.Input];
        public string ScaraProductId => slots[ProductLocation.Scara];
        public string HandoffProductId => slots[ProductLocation.Handoff];
        public string FanucProductId => slots[ProductLocation.Fanuc];
        public IReadOnlyList<ProductRecord> Snapshot() => registry.Snapshot();
        public IReadOnlyList<string> WaitingSnapshot() => waiting.ToArray();
        public bool TryGet(string id, out ProductRecord product) => registry.TryGet(id, out product);

        public bool TryEnqueue(string id, ProductDestination destination, out ProductRecord product, out string error)
        {
            if (!registry.TryRegister(id, destination, out product, out error)) return false;
            waiting.Enqueue(id);
            return true;
        }

        public bool TryFeedNext(out ProductRecord product, out string error)
        {
            product = null; error = null;
            if (InputProductId != null) { error = "투입 지점 사용 중: 대기열을 유지합니다."; return false; }
            if (waiting.Count == 0) { error = "대기 중인 상품이 없습니다."; return false; }
            string id = waiting.Peek();
            if (!registry.TryTransition(id, ProductLocation.Waiting, ProductLocation.Input, out product, out error)) return false;
            waiting.Dequeue();
            slots[ProductLocation.Input] = id;
            return true;
        }

        public bool TryTransfer(string id, ProductLocation expected, ProductLocation next, out ProductRecord product, out string error)
        {
            product = null; error = null;
            if (expected == ProductLocation.Waiting || next == ProductLocation.Waiting || next == ProductLocation.Input || next == ProductLocation.Removed)
            { error = "대기 상품의 투입/제거는 전용 요청으로 처리해야 합니다."; return false; }
            if (!registry.TryGet(id, out var current)) { error = "등록되지 않은 상품입니다."; return false; }
            if (current.Location != expected) { error = "상품의 현재 상태가 요청 시점과 다릅니다."; return false; }
            if (slots.TryGetValue(expected, out var source) && source != id) { error = "상품 상태와 출발 지점의 점유가 맞지 않습니다."; return false; }
            if (slots.TryGetValue(next, out var occupant) && occupant != null) { error = "도착 지점 사용 중: 현재 보유 상태를 유지합니다."; return false; }
            if (!registry.TryTransition(id, expected, next, out product, out error)) return false;
            if (slots.ContainsKey(expected)) slots[expected] = null;
            if (slots.ContainsKey(next)) slots[next] = id;
            return true;
        }

        // Physical destruction and explicit operator intent are checked by the tracker.
        // Keep the ID/receipt while releasing only this record's FIFO/slot occupancy.
        public bool TryConfirmLoss(string id, ProductLocation expected, string reason, out ProductRecord product, out string error)
        {
            product = null; error = null;
            if (!registry.TryGet(id, out var current)) { error = "등록되지 않은 상품입니다."; return false; }
            if (current.Location != expected) { error = "상품의 현재 상태가 요청 시점과 다릅니다."; return false; }
            if (slots.TryGetValue(expected, out var source) && source != id) { error = "상품 상태와 손실 지점의 점유가 맞지 않습니다."; return false; }
            var remaining = expected == ProductLocation.Waiting ? new Queue<string>(waiting.Where(queued => queued != id)) : waiting;
            if (!registry.TryConfirmLoss(id, expected, reason, out product, out error)) return false;
            waiting = remaining;
            if (slots.ContainsKey(expected)) slots[expected] = null;
            return true;
        }

        public bool TryRemove(string id, ProductLocation expected, out ProductRecord product, out string error)
        {
            product = null; error = null;
            if (!registry.TryGet(id, out var current)) { error = "등록되지 않은 상품입니다."; return false; }
            if (current.Location != expected) { error = "상품의 현재 상태가 요청 시점과 다릅니다."; return false; }
            if (slots.TryGetValue(expected, out var source) && source != id) { error = "상품 상태와 제거 지점의 점유가 맞지 않습니다."; return false; }
            var remaining = expected == ProductLocation.Waiting ? new Queue<string>(waiting.Where(queued => queued != id)) : waiting;
            if (!registry.TryTransition(id, expected, ProductLocation.Removed, out product, out error)) return false;
            waiting = remaining;
            if (slots.ContainsKey(expected)) slots[expected] = null;
            return true;
        }
    }
}
