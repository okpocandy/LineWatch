using System;
using UnityEngine;

namespace LineWatch
{
    // Draft input only. Actual movement goes through the existing cell and robot command gates.
    public sealed class CellManualTcpCommand
    {
        private readonly CellEquipmentSelection selection;
        private uint selectionRevision;
        public uint DraftVersion { get; private set; }
        public bool HasDraft { get; private set; }
        public string DraftEquipmentId { get; private set; }
        public Vector3 PositionMillimetres { get; private set; }
        public Vector3 RotationDegrees { get; private set; }
        public bool? LastRequestAccepted { get; private set; }
        public string LastRequestMessage { get; private set; }

        public CellManualTcpCommand(CellEquipmentSelection selection)
        {
            this.selection = selection ?? throw new ArgumentNullException(nameof(selection));
            selectionRevision = selection.SelectionRevision;
        }
        public void SynchronizeSelection()
        {
            if (selectionRevision == selection.SelectionRevision) return;
            selectionRevision = selection.SelectionRevision; HasDraft = false; DraftEquipmentId = null;
            PositionMillimetres = RotationDegrees = Vector3.zero;
            LastRequestAccepted = null; LastRequestMessage = null; DraftVersion++;
        }
        public void SetDraft(Vector3 positionMillimetres, Vector3 rotationDegrees)
        {
            SynchronizeSelection();
            PositionMillimetres = positionMillimetres; RotationDegrees = rotationDegrees;
            DraftEquipmentId = selection.SelectedId; HasDraft = true; DraftVersion++;
            LastRequestAccepted = null; LastRequestMessage = null;
        }
        public bool TryLoadCurrent(out string error)
        {
            if (!Application.isPlaying) { error = "Play 후 현재 TCP를 불러오세요."; return LoadFailed(error); }
            if (!TryReadRobot(out var robot, out _, out error)) return LoadFailed(error);
            if (robot is ScaraJointLab s)
            {
                if (!s.IsReady) { error = "SCARA 연결/활성 상태를 확인하세요."; return LoadFailed(error); }
                SetDraft(s.ActualTcpPosition * 1000f, new Vector3(0, s.ActualTcpYawDegrees, 0));
            }
            else
            {
                var f = (FanucJointLab)robot;
                if (!f.TryReadActualTcp(out var position, out var rotation, out error)) return LoadFailed(error);
                SetDraft(position * 1000f, rotation.eulerAngles);
            }
            LastRequestMessage = "현재 TCP를 불러왔습니다. 이동은 요청하지 않았습니다.";
            error = null; return true;
        }
        private bool LoadFailed(string error)
        {
            HasDraft = false; DraftEquipmentId = null; DraftVersion++;
            LastRequestAccepted = false; LastRequestMessage = error; return false;
        }

        public bool CanExecute(out string error)
        {
            if (!Application.isPlaying) { error = "Play에서 수동 목표를 입력하세요."; return false; }
            if (!HasDraft || selectionRevision != selection.SelectionRevision || DraftEquipmentId != selection.SelectedId)
            { error = "현재 선택 로봇의 TCP를 불러오거나 목표를 입력하세요. 선택 변경 시 이전 입력은 사용하지 않습니다."; return false; }
            if (!Finite(PositionMillimetres) || !Finite(RotationDegrees))
            { error = "위치와 각도에 유한한 숫자를 입력하세요."; return false; }
            if (!TryReadRobot(out var robot, out var cell, out error)) return false;
            return cell.CanMoveManualTcp(robot, PositionMillimetres * .001f, out error);
        }
        public bool TryExecute(out string error)
        {
            if (!CanExecute(out error) || !TryReadRobot(out var robot, out _, out error))
            { LastRequestAccepted = false; LastRequestMessage = error; return false; }
            var position = PositionMillimetres * .001f;
            bool accepted = robot is ScaraJointLab s
                ? s.TryMoveTcp(position, RotationDegrees.y, ScaraJointLab.IkSelection.Nearest, out _, out error)
                : ((FanucJointLab)robot).TryMoveTcp(position, Quaternion.Euler(RotationDegrees), FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error);
            LastRequestAccepted = accepted;
            LastRequestMessage = accepted ? "수동 이동 요청 수락 · 실제 도달은 TCP 진단에서 확인하세요." : error;
            return accepted;
        }
        private bool TryReadRobot(out Component robot, out CellRunController cell, out string error)
        {
            robot = null; cell = null;
            if (!selection.TryGetSelected(out var selected) || selected.Kind == CellEquipmentSelection.EquipmentKind.Handoff)
            { error = "연결된 SCARA 또는 FANUC를 선택하세요."; return false; }
            robot = selected.Kind == CellEquipmentSelection.EquipmentKind.Scara ? (Component)selected.Target.GetComponent<ScaraJointLab>() : selected.Target.GetComponent<FanucJointLab>();
            cell = selected.Target.GetComponentInParent<CellRunController>(true);
            if (robot == null || cell == null) { error = "선택 로봇과 셀 연결을 확인하세요."; return false; }
            error = null; return true;
        }
        private static bool Finite(Vector3 v) => Finite(v.x) && Finite(v.y) && Finite(v.z);
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
