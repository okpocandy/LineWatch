using System;
using System.Collections.Generic;
using System.Linq;

namespace LineWatch
{
    public enum ProductDestination { A, B }
    public enum ProductLocation { Waiting, Input, Scara, Handoff, Fanuc, OutputA, OutputB, Removed, Lost }

    public sealed class ProductRecord
    {
        public string Id { get; }
        public ProductDestination Destination { get; }
        public ProductLocation Location { get; }
        public ProductLocation? LostFrom { get; }
        public string LossReason { get; }

        internal ProductRecord(string id, ProductDestination destination, ProductLocation location, ProductLocation? lostFrom = null, string lossReason = null)
        {
            Id = id; Destination = destination; Location = location;
            LostFrom = lostFrom; LossReason = lossReason;
        }
    }

    // Logical product records only. Scene objects, physical grip and station capacity are connected separately.
    public sealed class ProductRegistry
    {
        private readonly Dictionary<string, ProductRecord> products = new Dictionary<string, ProductRecord>(StringComparer.Ordinal);

        public int Count => products.Count;
        public IReadOnlyList<ProductRecord> Snapshot() => products.Values.ToArray();

        public bool TryGet(string id, out ProductRecord product)
        {
            product = null;
            return id != null && products.TryGetValue(id, out product);
        }

        public bool TryRegister(string id, ProductDestination destination, out ProductRecord product, out string error)
        {
            product = null; error = null;
            if (string.IsNullOrWhiteSpace(id) || id != id.Trim()) { error = "앞뒤 공백 없는 상품 ID가 필요합니다."; return false; }
            if (destination != ProductDestination.A && destination != ProductDestination.B) { error = "상품 목적지는 A 또는 B여야 합니다."; return false; }
            if (products.ContainsKey(id)) { error = "이미 등록된 상품 ID입니다."; return false; }
            product = new ProductRecord(id, destination, ProductLocation.Waiting);
            products.Add(id, product);
            return true;
        }

        public bool TryTransition(string id, ProductLocation expected, ProductLocation next, out ProductRecord product, out string error)
        {
            product = null; error = null;
            if (!Enum.IsDefined(typeof(ProductLocation), expected) || !Enum.IsDefined(typeof(ProductLocation), next))
            { error = "정의된 상품 상태가 필요합니다."; return false; }
            if (!TryGet(id, out var current)) { error = "등록되지 않은 상품입니다."; return false; }
            if (current.Location != expected) { error = "상품의 현재 상태가 요청 시점과 다릅니다."; return false; }
            if (!Allowed(current, next)) { error = "상품의 이동 순서 또는 목적지가 맞지 않습니다."; return false; }
            product = new ProductRecord(id, current.Destination, next);
            products[id] = product;
            return true;
        }

        public bool TryConfirmLoss(string id, ProductLocation expected, string reason, out ProductRecord product, out string error)
        {
            product = null; error = null;
            if (string.IsNullOrWhiteSpace(reason)) { error = "손실 확정 사유가 필요합니다."; return false; }
            if (!TryGet(id, out var current)) { error = "등록되지 않은 상품입니다."; return false; }
            if (current.Location != expected) { error = "상품의 현재 상태가 요청 시점과 다릅니다."; return false; }
            if (expected != ProductLocation.Waiting && expected != ProductLocation.Input && expected != ProductLocation.Scara &&
                expected != ProductLocation.Handoff && expected != ProductLocation.Fanuc)
            { error = "완료/제거/손실 확정 이력은 다시 정리하지 않습니다."; return false; }
            product = new ProductRecord(id, current.Destination, ProductLocation.Lost, expected, reason.Trim());
            products[id] = product;
            return true;
        }

        internal bool TryReturnToQueue(string id, ProductLocation expected, out ProductRecord product, out string error)
        {
            product = null; error = null;
            if (!TryGet(id, out var current) || current.Location != expected ||
                expected != ProductLocation.Input && expected != ProductLocation.Scara && expected != ProductLocation.Handoff && expected != ProductLocation.Fanuc)
            { error = "복구할 상품의 현재 점유 상태를 확인하세요."; return false; }
            product = new ProductRecord(id, current.Destination, ProductLocation.Waiting);
            products[id] = product;
            return true;
        }

        private static bool Allowed(ProductRecord product, ProductLocation next)
        {
            if (next == ProductLocation.Removed)
                return product.Location == ProductLocation.Waiting || product.Location == ProductLocation.Input || product.Location == ProductLocation.Handoff;
            switch (product.Location)
            {
                case ProductLocation.Waiting: return next == ProductLocation.Input;
                case ProductLocation.Input: return next == ProductLocation.Scara;
                case ProductLocation.Scara: return next == ProductLocation.Handoff;
                case ProductLocation.Handoff: return next == ProductLocation.Fanuc;
                case ProductLocation.Fanuc: return next == (product.Destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB);
                default: return false;
            }
        }
    }
}
