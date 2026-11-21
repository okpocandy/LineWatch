using System;
using UnityEngine;

namespace LineWatch
{
    // Selection is view state only. This catalog issues no robot, product or camera commands.
    public sealed class CellEquipmentSelection
    {
        public enum EquipmentKind { Scara, Fanuc, Handoff }
        public const string HandoffId = "HANDOFF-01";

        public readonly struct EquipmentInfo
        {
            private readonly Transform cell;
            public string Id { get; }
            public EquipmentKind Kind { get; }
            public int AxisCount { get; }
            public Transform Target { get; }
            public bool IsAvailable => !string.IsNullOrWhiteSpace(Id) && cell != null && Target != null && Target.IsChildOf(cell);

            internal EquipmentInfo(string id, EquipmentKind kind, int axes, Transform target, Transform cell)
            { Id = id; Kind = kind; AxisCount = axes; Target = target; this.cell = cell; }
        }

        private readonly LineWatchCellLayout layout;
        public string SelectedId { get; private set; }
        public string IdSearch { get; private set; } = "";
        public EquipmentKind? KindFilter { get; private set; }

        public CellEquipmentSelection(LineWatchCellLayout layout)
        {
            if (layout == null) throw new ArgumentNullException(nameof(layout));
            this.layout = layout;
        }

        public EquipmentInfo[] Snapshot()
        {
            if (layout == null) return Array.Empty<EquipmentInfo>();
            return new[]
            {
                new EquipmentInfo(layout.ScaraId, EquipmentKind.Scara, 4, layout.Scara != null ? layout.Scara.transform : null, layout.transform),
                new EquipmentInfo(layout.FanucId, EquipmentKind.Fanuc, 6, layout.Fanuc != null ? layout.Fanuc.transform : null, layout.transform),
                new EquipmentInfo(HandoffId, EquipmentKind.Handoff, 0, layout.HandoffPoint, layout.transform)
            };
        }

        public bool TrySelect(string id, out string error)
        {
            if (!TryRead(id, out var entry, out error)) return false;
            SelectedId = entry.Id;
            return true;
        }

        public bool TryGetSelected(out EquipmentInfo entry) => TryRead(SelectedId, out entry, out _);
        public void ClearSelection() => SelectedId = null;

        public void SetFilter(string idSearch, EquipmentKind? kind)
        {
            IdSearch = (idSearch ?? "").Trim();
            KindFilter = kind;
        }

        public bool MatchesFilter(EquipmentInfo entry) => (!KindFilter.HasValue || entry.Kind == KindFilter.Value)
            && (entry.Id ?? "").IndexOf(IdSearch, StringComparison.OrdinalIgnoreCase) >= 0;

        public bool TrySelectTarget(Transform target, out string error)
        {
            if (layout != null && target != null && target.IsChildOf(layout.transform) && target.GetComponentInParent<GrippableItem>(true) == null)
            {
                foreach (var entry in Snapshot())
                {
                    bool matches = entry.Kind == EquipmentKind.Handoff
                        ? IsWithin(target, layout.HandoffEquipment) || IsWithin(target, layout.HandoffZone) || IsWithin(target, layout.HandoffPoint)
                        : IsWithin(target, entry.Target);
                    if (matches) return TrySelect(entry.Id, out error);
                }
            }
            error = "현재 셀의 SCARA, FANUC 또는 인계대 형상을 선택하세요.";
            return false;
        }

        public bool TryGetSelectedBounds(out Bounds bounds)
        {
            bounds = default;
            if (!TryGetSelected(out var entry)) return false;
            var target = entry.Kind == EquipmentKind.Handoff
                ? (layout.HandoffEquipment != null ? layout.HandoffEquipment : layout.HandoffZone) : entry.Target;
            if (target == null || !target.IsChildOf(layout.transform)) return false;
            bool found = false;
            foreach (var renderer in target.GetComponentsInChildren<Renderer>())
            {
                if (!renderer.enabled || renderer.GetComponentInParent<GrippableItem>(true) != null) continue;
                if (found) bounds.Encapsulate(renderer.bounds);
                else { bounds = renderer.bounds; found = true; }
            }
            return found;
        }

        private static bool IsWithin(Transform target, Transform root) => root != null && target.IsChildOf(root);

        private bool TryRead(string id, out EquipmentInfo entry, out string error)
        {
            entry = default;
            if (string.IsNullOrWhiteSpace(id)) { error = "선택할 설비 ID가 필요합니다."; return false; }
            int matches = 0;
            foreach (var candidate in Snapshot())
                if (candidate.Id == id) { entry = candidate; matches++; }
            if (matches != 1 || !entry.IsAvailable)
            {
                entry = default;
                error = "현재 셀에 유일하게 연결된 설비 ID를 선택하세요.";
                return false;
            }
            error = null;
            return true;
        }
    }
}
