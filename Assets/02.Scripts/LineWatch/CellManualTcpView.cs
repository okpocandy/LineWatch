using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch
{
    public sealed class CellManualTcpView : VisualElement
    {
        private readonly CellEquipmentSelection selection;
        private readonly Foldout panel;
        private readonly FloatField x, y, z, yaw, rx, ry, rz;
        private readonly Button load, move;
        private readonly HelpBox eligibility, result;
        private readonly Label target;
        private uint shownVersion = uint.MaxValue, shownSelectionRevision;
        public CellManualTcpCommand Command { get; }

        public CellManualTcpView(CellEquipmentSelection selection)
        {
            this.selection = selection; Command = new CellManualTcpCommand(selection);
            name = "equipment-manual-tcp";
            panel = new Foldout { name = "equipment-manual-panel", text = "수동 TCP 목표", value = false }; Add(panel);
            panel.Add(new HelpBox("RobotFrame 기준 위치 mm / 각도 °입니다. 셀 정지·두 집게 빈 보유·자기 작업구역 또는 빈 공유 구역에서 요청합니다. 현재 위치가 구역 밖이면 불러온 값도 차단됩니다.", HelpBoxMessageType.Info));
            target = new Label { name = "equipment-manual-target" }; panel.Add(target);
            load = new Button(() => { Command.TryLoadCurrent(out _); Refresh(); }) { name = "equipment-manual-load", text = "현재 TCP 불러오기 · 이동 없음" }; panel.Add(load);
            x = Field("X (mm)", "x"); y = Field("Y (mm)", "y"); z = Field("Z (mm)", "z");
            yaw = Field("수평각 (°)", "yaw");
            rx = Field("회전 X (°)", "rx"); ry = Field("회전 Y (°)", "ry"); rz = Field("회전 Z (°)", "rz");
            foreach (var field in new[] { x, y, z, yaw, rx, ry, rz }) field.RegisterValueChangedCallback(_ => EditDraft());
            eligibility = new HelpBox("", HelpBoxMessageType.Info) { name = "equipment-manual-eligibility" }; panel.Add(eligibility);
            move = new Button(() => { Command.TryExecute(out _); Refresh(); }) { name = "equipment-manual-move", text = "수동 TCP 이동 요청" }; panel.Add(move);
            result = new HelpBox("", HelpBoxMessageType.Info) { name = "equipment-manual-result" }; panel.Add(result);
            Refresh();
        }
        private FloatField Field(string label, string suffix)
        {
            var field = new FloatField(label) { name = "equipment-manual-" + suffix };
            panel.Add(field); return field;
        }
        private void EditDraft()
        {
            if (shownSelectionRevision != selection.SelectionRevision) { Refresh(); return; }
            if (!selection.TryGetSelected(out var selected)) return;
            Command.SetDraft(new Vector3(x.value, y.value, z.value), selected.Kind == CellEquipmentSelection.EquipmentKind.Scara
                ? new Vector3(0, yaw.value, 0) : new Vector3(rx.value, ry.value, rz.value));
            shownVersion = Command.DraftVersion; // Do not rewrite the field being edited.
            Refresh();
        }
        public void Refresh()
        {
            Command.SynchronizeSelection(); shownSelectionRevision = selection.SelectionRevision;
            bool robot = selection.TryGetSelected(out var selected) && selected.Kind != CellEquipmentSelection.EquipmentKind.Handoff;
            style.display = robot ? DisplayStyle.Flex : DisplayStyle.None;
            if (!robot) { move.SetEnabled(false); return; }
            target.text = "대상: " + selected.Id + " · RobotFrame 로컬";
            bool scara = selected.Kind == CellEquipmentSelection.EquipmentKind.Scara;
            yaw.style.display = scara ? DisplayStyle.Flex : DisplayStyle.None;
            foreach (var field in new[] { rx, ry, rz }) field.style.display = scara ? DisplayStyle.None : DisplayStyle.Flex;
            if (shownVersion != Command.DraftVersion)
            {
                var p = Command.PositionMillimetres; var r = Command.RotationDegrees;
                x.SetValueWithoutNotify(p.x); y.SetValueWithoutNotify(p.y); z.SetValueWithoutNotify(p.z); yaw.SetValueWithoutNotify(r.y);
                rx.SetValueWithoutNotify(r.x); ry.SetValueWithoutNotify(r.y); rz.SetValueWithoutNotify(r.z);
                shownVersion = Command.DraftVersion;
            }
            load.SetEnabled(Application.isPlaying);
            foreach (var field in new[] { x, y, z, yaw, rx, ry, rz }) field.SetEnabled(Application.isPlaying);
            bool allowed = Command.CanExecute(out var reason);
            move.SetEnabled(allowed);
            eligibility.text = allowed ? "요청 가능 · IK와 경로/상태는 실행 시 다시 검사합니다." : reason;
            eligibility.messageType = allowed ? HelpBoxMessageType.Info : HelpBoxMessageType.Warning;
            result.text = Command.LastRequestMessage ?? "";
            result.messageType = Command.LastRequestAccepted == false ? HelpBoxMessageType.Warning : HelpBoxMessageType.Info;
            result.style.display = string.IsNullOrEmpty(result.text) ? DisplayStyle.None : DisplayStyle.Flex;
        }
    }
}
