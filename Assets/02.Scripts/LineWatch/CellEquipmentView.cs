using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch
{
    // UI Toolkit view shared by hosts. Equipment selection never dispatches motion.
    public sealed class CellEquipmentView : VisualElement
    {
        private readonly CellEquipmentSelection selection;
        private readonly CellEquipmentSelection.EquipmentInfo[] entries;
        private readonly Button[] choices;
        private readonly Button clear;
        private readonly TextField searchField;
        private readonly DropdownField kindFilter;
        private readonly Label filterSummary;
        private readonly Label summary, id, kind, axes, cellState, equipmentState, step, product;
        private readonly HelpBox request;
        private string requestMessage;

        public CellEquipmentView(CellEquipmentSelection selection)
        {
            if (selection == null) throw new ArgumentNullException(nameof(selection));
            this.selection = selection;
            name = "cell-equipment-view";
            style.paddingLeft = style.paddingRight = 6;
            summary = Detail("equipment-selection-summary"); summary.style.unityFontStyleAndWeight = FontStyle.Bold;
            searchField = new TextField("ID 검색") { name = "equipment-id-search" };
            kindFilter = new DropdownField("기종", new List<string> { "전체", "SCARA", "FANUC", "인계대" }, 0) { name = "equipment-kind-filter" };
            Add(searchField); Add(kindFilter);
            searchField.RegisterValueChangedCallback(_ => ApplyFilter());
            kindFilter.RegisterValueChangedCallback(_ => ApplyFilter());
            filterSummary = Detail("equipment-filter-summary");
            entries = selection.Snapshot(); choices = new Button[entries.Length];
            for (int i = 0; i < entries.Length; i++)
            {
                var entry = entries[i];
                var button = new Button(() =>
                {
                    requestMessage = selection.TrySelect(entry.Id, out var error) ? null : error;
                    Refresh();
                }) { name = "equipment-select-" + entry.Kind.ToString().ToLowerInvariant(),
                    text = entry.Id + " · " + KindText(entry.Kind) + (entry.AxisCount > 0 ? " · " + entry.AxisCount + "축" : "") };
                button.style.minHeight = 28; button.style.marginTop = 4; button.style.unityTextAlign = TextAnchor.MiddleLeft;
                choices[i] = button; Add(button);
            }
            clear = new Button(() => { selection.ClearSelection(); requestMessage = null; Refresh(); })
                { name = "equipment-clear", text = "선택 해제" };
            clear.style.minHeight = 28; clear.style.marginTop = 6; Add(clear);
            id = Detail("equipment-detail-id"); kind = Detail("equipment-detail-kind"); axes = Detail("equipment-detail-axes");
            cellState = Detail("equipment-detail-cell-state"); equipmentState = Detail("equipment-detail-state");
            step = Detail("equipment-detail-step"); product = Detail("equipment-detail-product");
            request = new HelpBox("", HelpBoxMessageType.Warning) { name = "equipment-selection-request" }; Add(request);
            Refresh(); schedule.Execute(Refresh).Every(100);
        }

        private void ApplyFilter()
        {
            selection.SetFilter(searchField.value, kindFilter.index > 0 ? (CellEquipmentSelection.EquipmentKind?)(kindFilter.index - 1) : null);
            Refresh();
        }

        private Label Detail(string name)
        {
            var label = new Label { name = name }; label.style.whiteSpace = WhiteSpace.Normal; label.style.marginTop = 4;
            Add(label); return label;
        }

        public void Refresh()
        {
            searchField.SetValueWithoutNotify(selection.IdSearch);
            kindFilter.SetValueWithoutNotify(kindFilter.choices[selection.KindFilter.HasValue ? (int)selection.KindFilter.Value + 1 : 0]);
            int visible = 0;
            for (int i = 0; i < choices.Length; i++)
            {
                bool matches = selection.MatchesFilter(entries[i]);
                choices[i].style.display = matches ? DisplayStyle.Flex : DisplayStyle.None;
                if (matches) visible++;
                choices[i].SetEnabled(entries[i].IsAvailable);
                choices[i].style.backgroundColor = selection.SelectedId == entries[i].Id
                    ? new StyleColor(new Color(.18f, .34f, .5f, .8f)) : new StyleColor(StyleKeyword.Null);
            }
            filterSummary.text = visible == 0 ? "검색 결과 없음 (0/" + entries.Length + ")" : "목록: " + visible + "/" + entries.Length;
            clear.SetEnabled(selection.SelectedId != null);
            request.text = requestMessage ?? ""; request.style.display = string.IsNullOrEmpty(requestMessage) ? DisplayStyle.None : DisplayStyle.Flex;
            if (!selection.TryGetSelected(out var selected))
            {
                summary.text = selection.SelectedId == null ? "선택된 설비 없음" : "선택 연결 확인 필요 · " + selection.SelectedId;
                id.text = "ID: -"; kind.text = "기종: -"; axes.text = "축 수: -"; cellState.text = "셀 운전: -";
                equipmentState.text = "설비 상태: -"; step.text = "로봇 단계: -"; product.text = "상품: -";
                return;
            }
            summary.text = "선택: " + selected.Id + (selection.MatchesFilter(selected) ? "" : " · 목록 필터로 숨김");
            id.text = "ID: " + selected.Id; kind.text = "기종: " + KindText(selected.Kind); axes.text = "축 수: " + selected.AxisCount;
            var cell = selected.Target.GetComponentInParent<CellRunController>(true);
            cellState.text = "셀 운전: " + (!Application.isPlaying ? "Play 대기" : cell == null ? "연결 없음" : cell.State.ToString());
            if (selected.Kind == CellEquipmentSelection.EquipmentKind.Handoff)
            {
                var tracker = cell != null ? cell.GetComponent<CellProductTracker>() : null;
                equipmentState.text = "설비 상태: " + (!Application.isPlaying ? "Play 대기" : tracker == null ? "연결 없음" : tracker.IsFaulted ? "추적 고장" : tracker.HandoffProductId == null ? "비어 있음" : "점유");
                step.text = "로봇 단계: 해당 없음"; product.text = "인계 상품: " + (tracker != null ? tracker.HandoffProductId ?? "-" : "확인 불가");
            }
            else
            {
                var sequence = selected.Target.GetComponent<RobotWaypointSequence>(); var gripper = selected.Target.GetComponent<RobotGripper>();
                equipmentState.text = "설비 상태: " + (!Application.isPlaying ? "Play 대기" : !selected.Target.gameObject.activeInHierarchy ? "비활성" :
                    sequence == null || gripper == null ? "연결 없음" : !sequence.IsReady || !gripper.IsReady ? "준비 안 됨" : sequence.State.ToString());
                step.text = "로봇 단계: " + (!Application.isPlaying || sequence == null ? "-" : sequence.State == RobotWaypointSequence.RunState.Idle ? "대기" : sequence.StepName);
                var held = gripper != null ? gripper.HeldItem : null;
                product.text = "실제 보유 상품: " + (gripper == null ? "확인 불가" : held == null ? "-" : held.Owner != gripper ? "보유 불일치" : held.ProductId ?? "미등록 시험 물체");
            }
        }

        private static string KindText(CellEquipmentSelection.EquipmentKind kind) => kind == CellEquipmentSelection.EquipmentKind.Scara ? "SCARA"
            : kind == CellEquipmentSelection.EquipmentKind.Fanuc ? "FANUC" : "인계대";
    }
}
