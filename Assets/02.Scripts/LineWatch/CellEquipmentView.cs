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
        private readonly Foldout diagnostics;
        private readonly Label diagnosticState, actualTcp, calculatedTcp, fkError, jointValues;
        private readonly Label acceptedTarget, targetError, ikState, tcpRequest, motionError;
        private readonly HelpBox request;
        private readonly CellManualTcpView manualTcp;
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
            diagnostics = new Foldout { name = "equipment-diagnostics", text = "TCP · 관절 진단", value = false }; Add(diagnostics);
            diagnosticState = Detail("equipment-diagnostic-state", diagnostics);
            actualTcp = Detail("equipment-diagnostic-actual", diagnostics);
            calculatedTcp = Detail("equipment-diagnostic-calculated", diagnostics);
            fkError = Detail("equipment-diagnostic-error", diagnostics);
            jointValues = Detail("equipment-diagnostic-joints", diagnostics);
            acceptedTarget = Detail("equipment-diagnostic-target", diagnostics);
            targetError = Detail("equipment-diagnostic-target-error", diagnostics);
            ikState = Detail("equipment-diagnostic-ik", diagnostics);
            tcpRequest = Detail("equipment-diagnostic-request", diagnostics);
            motionError = Detail("equipment-diagnostic-motion-error", diagnostics);
            manualTcp = new CellManualTcpView(selection); Add(manualTcp);
            request = new HelpBox("", HelpBoxMessageType.Warning) { name = "equipment-selection-request" }; Add(request);
            Refresh(); schedule.Execute(Refresh).Every(100);
        }

        private void ApplyFilter()
        {
            selection.SetFilter(searchField.value, kindFilter.index > 0 ? (CellEquipmentSelection.EquipmentKind?)(kindFilter.index - 1) : null);
            Refresh();
        }

        private Label Detail(string name, VisualElement parent = null)
        {
            var label = new Label { name = name }; label.style.whiteSpace = WhiteSpace.Normal; label.style.marginTop = 4;
            (parent ?? this).Add(label); return label;
        }

        public void Refresh()
        {
            manualTcp.Refresh();
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
                ClearDiagnostics("로봇 선택 후 표시"); diagnostics.style.display = DisplayStyle.None;
                return;
            }
            summary.text = "선택: " + selected.Id + (selection.MatchesFilter(selected) ? "" : " · 목록 필터로 숨김");
            id.text = "ID: " + selected.Id; kind.text = "기종: " + KindText(selected.Kind); axes.text = "축 수: " + selected.AxisCount;
            var cell = selected.Target.GetComponentInParent<CellRunController>(true);
            cellState.text = "셀 운전: " + (!Application.isPlaying ? "Play 대기" : cell == null ? "연결 없음" : cell.State.ToString());
            RefreshDiagnostics(selected);
            if (selected.Kind == CellEquipmentSelection.EquipmentKind.Handoff)
            {
                var tracker = cell != null ? cell.GetComponent<CellProductTracker>() : null;
                equipmentState.text = "설비 상태: " + (!Application.isPlaying ? "Play 대기" : tracker == null ? "연결 없음" : tracker.IsFaulted ? "추적 고장" : tracker.HandoffProductId == null ? "비어 있음" : "점유");
                step.text = "로봇 단계: 해당 없음"; product.text = "인계 상품: " + (tracker != null ? tracker.HandoffProductId ?? "-" : "확인 불가");
            }
            else
            {
                var sequence = selected.Target.GetComponent<RobotWaypointSequence>(); var gripper = selected.Target.GetComponent<RobotGripper>();
                var s = selected.Target.GetComponent<ScaraJointLab>(); var f = selected.Target.GetComponent<FanucJointLab>();
                bool tcpMoving = (s != null && s.IsTrajectoryMoving) || (f != null && f.IsTcpMoving);
                equipmentState.text = "설비 상태: " + (!Application.isPlaying ? "Play 대기" : !selected.Target.gameObject.activeInHierarchy ? "비활성" :
                    sequence == null || gripper == null ? "연결 없음" : !sequence.IsReady || !gripper.IsReady ? "준비 안 됨"
                        : !sequence.IsBusy && tcpMoving ? "수동 TCP 이동" : sequence.State.ToString());
                step.text = "로봇 단계: " + (!Application.isPlaying || sequence == null ? "-" : sequence.State == RobotWaypointSequence.RunState.Idle ? "대기" : sequence.StepName);
                var held = gripper != null ? gripper.HeldItem : null;
                product.text = "실제 보유 상품: " + (gripper == null ? "확인 불가" : held == null ? "-" : held.Owner != gripper ? "보유 불일치" : held.ProductId ?? "미등록 시험 물체");
            }
        }

        private void ClearDiagnostics(string message)
        {
            diagnosticState.text = message;
            actualTcp.text = "실제 TCP: -"; calculatedTcp.text = "FK 계산 TCP: -";
            fkError.text = "FK 대조 오차: -"; jointValues.text = "관절 현재 / 저장 목표: -";
            acceptedTarget.text = "최근 수락 목표 TCP: -"; targetError.text = "수락 목표 대비 실제 오차: -";
            ikState.text = "최근 이동 IK: -"; tcpRequest.text = "최근 TCP 요청: -"; motionError.text = "실행 중단 사유: -";
        }

        private void RefreshDiagnostics(CellEquipmentSelection.EquipmentInfo selected)
        {
            bool robot = selected.Kind != CellEquipmentSelection.EquipmentKind.Handoff;
            diagnostics.style.display = robot ? DisplayStyle.Flex : DisplayStyle.None;
            if (!robot) { ClearDiagnostics("인계대에는 로봇 진단이 없습니다."); return; }
            if (!Application.isPlaying) { ClearDiagnostics("Play 후 현재 로봇 진단을 표시합니다."); return; }
            if (selected.Kind == CellEquipmentSelection.EquipmentKind.Scara)
            {
                var s = selected.Target.GetComponent<ScaraJointLab>();
                if (s == null) { ClearDiagnostics("SCARA 연결 확인 필요"); return; }
                if (!s.TryReadFkDiagnostics(out var d, out var error))
                { ClearDiagnostics("진단 읽기 실패: " + error); RefreshScaraRequest(s, null); return; }
                diagnosticState.text = "RobotFrame 기준 · 위치 mm · SCARA 수평각 °";
                actualTcp.text = "실제 TCP: " + Position(d.ActualPositionMetres) + " · 수평각 " + d.ActualYawDegrees.ToString("F3") + "°";
                calculatedTcp.text = "FK 계산 TCP: " + Position(d.Calculated.PositionMetres) + " · 수평각 " + d.Calculated.YawDegrees.ToString("F3") + "°";
                fkError.text = $"FK 대조 오차: 위치 {d.PositionErrorMetres * 1000f:F6} mm · 수평각 {d.YawErrorDegrees:F6}°";
                jointValues.text = $"관절 현재 / 저장 목표\nQ1: {s.Axis1.CurrentPosition:F3}° / {s.Axis1.TargetPosition:F3}°\nQ2: {s.Axis2.CurrentPosition:F3}° / {s.Axis2.TargetPosition:F3}°\nD3: {s.Axis3.CurrentPosition * 1000f:F3} mm / {s.Axis3.TargetPosition * 1000f:F3} mm\nQ4: {s.Axis4.CurrentPosition:F3}° / {s.Axis4.TargetPosition:F3}°";
                RefreshScaraRequest(s, d);
            }
            else
            {
                var f = selected.Target.GetComponent<FanucJointLab>();
                if (f == null) { ClearDiagnostics("FANUC 연결 확인 필요"); return; }
                if (!f.TryReadFkDiagnostics(out var d, out var error))
                { ClearDiagnostics("진단 읽기 실패: " + error); RefreshFanucRequest(f, null); return; }
                diagnosticState.text = "RobotFrame 기준 · 위치 mm · FANUC 방향 Euler XYZ °";
                actualTcp.text = "실제 TCP: " + Position(d.ActualPositionMetres) + " · 방향 " + d.ActualRotation.eulerAngles.ToString("F3");
                calculatedTcp.text = "FK 계산 TCP: " + Position(d.Calculated.PositionMetres) + " · 방향 " + d.Calculated.Rotation.eulerAngles.ToString("F3");
                fkError.text = $"FK 대조 오차: 위치 {d.PositionErrorMetres * 1000f:F6} mm · 전체 회전 {d.RotationErrorDegrees:F6}°";
                jointValues.text = "관절 현재 / 저장 목표";
                for (int i = 0; i < f.Joints.Count; i++)
                    jointValues.text += $"\nJ{i + 1}: {f.Joints[i].CurrentPosition:F3}° / {f.Joints[i].TargetPosition:F3}°";
                RefreshFanucRequest(f, d);
            }
        }

        private void RefreshScaraRequest(ScaraJointLab s, ScaraJointLab.FkDiagnostics? actual)
        {
            if (s.LastAcceptedTcpPosition.HasValue)
            {
                var goal = s.LastAcceptedTcpPosition.Value;
                acceptedTarget.text = "최근 수락 목표 TCP: " + Position(goal) + " · 수평각 " + s.LastAcceptedTcpYawDegrees.ToString("F3") + "°";
                targetError.text = actual.HasValue
                    ? $"수락 목표 대비 실제 오차: 위치 {Vector3.Distance(goal, actual.Value.ActualPositionMetres) * 1000f:F6} mm · 수평각 {Mathf.Abs(Mathf.DeltaAngle(s.LastAcceptedTcpYawDegrees, actual.Value.ActualYawDegrees)):F6}°"
                    : "수락 목표 대비 실제 오차: 실제 TCP 읽기 필요";
            }
            else { acceptedTarget.text = "최근 수락 목표 TCP: 없음"; targetError.text = "수락 목표 대비 실제 오차: -"; }
            var result = s.LastMoveIkResult;
            ikState.text = result == null ? "최근 이동 IK: 계산 기록 없음"
                : result.Success ? "최근 이동 IK: 해석해 있음 · 후보 " + result.Solutions.Count : "최근 이동 IK: 실패 · " + result.Failure;
            tcpRequest.text = RequestText(s.LastTcpRequestAccepted, s.LastTcpRequestError, result != null);
            motionError.text = "실행 중단 사유: " + (s.LastMotionError ?? "-");
        }

        private void RefreshFanucRequest(FanucJointLab f, FanucJointLab.FkDiagnostics? actual)
        {
            if (f.LastAcceptedTcpTarget.HasValue)
            {
                var goal = f.LastAcceptedTcpTarget.Value;
                acceptedTarget.text = "최근 수락 목표 TCP: " + Position(goal.position) + " · 방향 " + goal.rotation.eulerAngles.ToString("F3");
                targetError.text = actual.HasValue
                    ? $"수락 목표 대비 실제 오차: 위치 {Vector3.Distance(goal.position, actual.Value.ActualPositionMetres) * 1000f:F6} mm · 전체 회전 {Quaternion.Angle(goal.rotation, actual.Value.ActualRotation):F6}°"
                    : "수락 목표 대비 실제 오차: 실제 TCP 읽기 필요";
            }
            else { acceptedTarget.text = "최근 수락 목표 TCP: 없음"; targetError.text = "수락 목표 대비 실제 오차: -"; }
            var preview = f.LastMoveIkPreview;
            var result = preview.HasValue ? preview.Value.Result : null;
            ikState.text = result == null ? "최근 이동 IK: 계산 기록 없음"
                : (result.Succeeded ? "최근 이동 IK: 수렴" : "최근 이동 IK: 실패 · " + result.Reason)
                    + $" · 반복 {result.Iterations} · 초기값 {preview.Value.SeedName} · {result.ElapsedMilliseconds:F3} ms";
            tcpRequest.text = RequestText(f.LastTcpRequestAccepted, f.LastTcpRequestError, result != null);
            motionError.text = "실행 중단 사유: " + (f.LastMotionError ?? "-");
        }

        private static string RequestText(bool? accepted, string error, bool calculated) => !accepted.HasValue ? "최근 TCP 요청: 없음"
            : accepted.Value ? "최근 TCP 요청: 수락" : "최근 TCP 요청: 거부" + (calculated ? " · " : " · IK 계산 전 · ") + error;

        private static string Position(Vector3 metres) => $"({metres.x * 1000f:F3}, {metres.y * 1000f:F3}, {metres.z * 1000f:F3}) mm";

        private static string KindText(CellEquipmentSelection.EquipmentKind kind) => kind == CellEquipmentSelection.EquipmentKind.Scara ? "SCARA"
            : kind == CellEquipmentSelection.EquipmentKind.Fanuc ? "FANUC" : "인계대";
    }
}
