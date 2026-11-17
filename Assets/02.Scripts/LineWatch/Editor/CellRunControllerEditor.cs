using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    [CustomEditor(typeof(CellRunController))]
    public sealed class CellRunControllerEditor : UnityEditor.Editor
    {
        [MenuItem("LineWatch/셀 공통 조작")]
        private static void OpenCell()
        {
            var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작");
            window.minSize = new Vector2(440, 480);
            window.Show();
            window.RefreshCell();
        }

        public override VisualElement CreateInspectorGUI() => CreateControls((CellRunController)target);

        internal static bool ManualAllowed(Component target, out string error)
        {
            error = null;
            if (target == null) { error = "로봇 연결을 확인하세요."; return false; }
            var layout = target.GetComponentInParent<LineWatchCellLayout>(true);
            if (layout == null) return true;
            var cell = layout.GetComponent<CellRunController>();
            if (cell == null) { error = "셀 운전 관리자가 없습니다. 명령을 차단합니다."; return false; }
            return cell.CanIssueManualCommand(target, out error);
        }

        internal static VisualElement CreateControls(CellRunController cell)
        {
            var root = new VisualElement { name = "cell-run-controls" };
            var title = new Label("두 로봇 셀 · 공통 조작");
            title.style.fontSize = 16; title.style.unityFontStyleAndWeight = FontStyle.Bold; root.Add(title);
            if (cell == null) { root.Add(new HelpBox("LineWatch 씬의 셀 운전 관리자를 확인하세요.", HelpBoxMessageType.Warning)); return root; }
            var layout = cell.GetComponent<LineWatchCellLayout>();
            var scara = layout != null && layout.Scara != null ? layout.Scara.GetComponent<RobotWaypointSequence>() : null;
            var fanuc = layout != null && layout.Fanuc != null ? layout.Fanuc.GetComponent<RobotWaypointSequence>() : null;
            root.Add(new ObjectField("셀 운전 관리자") { name = "cell-run-controller", value = cell, objectType = typeof(CellRunController), allowSceneObjects = true });
            root.Q<ObjectField>().SetEnabled(false);
            var state = new Label { name = "cell-run-state" };
            var owner = new Label { name = "cell-run-owner" };
            var scaraState = new Label { name = "cell-run-scara" };
            var fanucState = new Label { name = "cell-run-fanuc" };
            var products = new Label { name = "cell-run-products" };
            foreach (var label in new[] { state, owner, scaraState, fanucState, products })
            { label.style.whiteSpace = WhiteSpace.Normal; label.style.marginTop = 6; root.Add(label); }
            var mode = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-run-mode" }; root.Add(mode);
            var fault = new HelpBox("", HelpBoxMessageType.Warning) { name = "cell-run-fault" }; root.Add(fault);
            var commands = new Dictionary<CellRunCommand, Button>();
            string requestMessage = null; bool requestAccepted = false;
            Button Command(CellRunCommand command, string name, string text)
            {
                var b = new Button(() =>
                {
                    if (cell == null) { requestAccepted = false; requestMessage = "셀 운전 관리자가 없습니다."; return; }
                    requestAccepted = cell.TryExecute(command, out var error);
                    requestMessage = requestAccepted ? text + " 요청 수락" : error;
                }) { name = name, text = text };
                commands.Add(command, b); return b;
            }
            var start = Command(CellRunCommand.Start, "cell-run-start", "시작");
            var pause = Command(CellRunCommand.Pause, "cell-run-pause", "일시정지");
            var resume = Command(CellRunCommand.Resume, "cell-run-resume", "재개");
            var stop = Command(CellRunCommand.Stop, "cell-run-stop", "정지 · 보유 유지");
            var reset = Command(CellRunCommand.Reset, "cell-run-reset", "고장 해제 · 기록 유지");
            Button Take(RobotWaypointSequence sequence, string name, string text) => new Button(() =>
            {
                if (cell == null) { requestAccepted = false; requestMessage = "셀 운전 관리자가 없습니다."; return; }
                requestAccepted = cell.TryTakeControl(sequence, out var error);
                requestMessage = requestAccepted ? text + " 수락 · 공통 시작으로 경로를 이어가세요." : error;
            }) { name = name, text = text };
            var takeScara = Take(scara, "cell-run-take-scara", "SCARA 경로 인계");
            var takeFanuc = Take(fanuc, "cell-run-take-fanuc", "FANUC 경로 인계");
            var release = new Button(() =>
            {
                if (cell == null) { requestAccepted = false; requestMessage = "셀 운전 관리자가 없습니다."; return; }
                requestAccepted = cell.TryReleaseControl(out var error);
                requestMessage = requestAccepted ? "권한 반환 수락 · 저장 경로와 보유는 유지됩니다." : error;
            }) { name = "cell-run-return", text = "개별 조작으로 권한 반환" };
            void Row(params Button[] buttons)
            {
                var row = new VisualElement(); row.style.flexDirection = FlexDirection.Row; row.style.flexWrap = Wrap.Wrap;
                foreach (var b in buttons)
                { b.style.flexGrow = 1; b.style.flexBasis = 0; b.style.minWidth = 100; b.style.minHeight = 28; b.style.marginTop = 5; row.Add(b); }
                root.Add(row);
            }
            Row(takeScara, takeFanuc); Row(start, pause, resume); Row(stop, reset); Row(release);
            var request = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-run-request" }; root.Add(request);
            root.Add(new HelpBox("개별 자동/순차 운전을 정지한 뒤 경로를 인계하세요. 공통 정지는 권한과 보유를 유지합니다. 개별 조작 복귀는 정지 후 권한 반환으로 구분합니다. 활성 버튼도 실행 직전 경로·정렬 검사에서 거부될 수 있습니다.", HelpBoxMessageType.Info));
            root.Add(new HelpBox("고장 해제는 실물·ID·보유/위치가 확인되는 셀/추적 고장과 셀이 인계받은 경로 고장을 명시 해제합니다. 자동 집기 경로는 집기 전 또는 같은 실물 보유 중의 단계/행동 횟수를 확인합니다. 경로/FK 검사 후 셀 Stopped·경로 Paused에서 대기하며 상품·통계·보유·단계·인계 권한을 유지합니다. 손실/보유 불일치·미인계/놓기 완료 후 경로 복구·데이터 초기화는 후속입니다.", HelpBoxMessageType.Info));
            void Refresh()
            {
                if (cell == null)
                {
                    foreach (var b in commands.Values.Concat(new[] { takeScara, takeFanuc, release })) b.SetEnabled(false);
                    state.text = "셀 상태: 관리자 없음 · 조작 차단"; owner.text = "명령 권한: 확인 불가";
                    scaraState.text = "SCARA: 관리자 없음 · 상태 확인 불가"; fanucState.text = "FANUC: 관리자 없음 · 상태 확인 불가";
                    products.text = "상품 추적: 별도 관리자와 실제 보유를 확인하세요.";
                    mode.text = "관리자 소실 후 개별 명령을 허용하지 않습니다.";
                    fault.text = "셀 운전 관리자가 사라졌습니다. 개별 조작으로 자동 전환하지 않습니다."; fault.style.display = DisplayStyle.Flex;
                    return;
                }
                bool playing = Application.isPlaying;
                state.text = playing ? "셀 상태: " + cell.State : "셀 상태: Play 대기";
                owner.text = "명령 권한: " + (!playing ? "Play 대기" : !cell.HasControlledSequence ? "개별 시험"
                    : cell.ControlledSequence == null ? "공통 제어 · 경로 연결 없음"
                    : cell.ControlledSequence == scara ? layout.ScaraId + " 경로 · 공통 제어"
                    : cell.ControlledSequence == fanuc ? layout.FanucId + " 경로 · 공통 제어" : "공통 제어 · 경로 연결 없음");
                scaraState.text = RobotText(layout != null ? layout.ScaraId : "SCARA", scara);
                fanucState.text = RobotText(layout != null ? layout.FanucId : "FANUC", fanuc);
                var tracker = cell.GetComponent<CellProductTracker>();
                products.text = tracker == null ? "상품 추적: 연결 없음" : $"상품 등록 기록: {tracker.Count} · 대기 {tracker.WaitingSnapshot().Count}\n투입 {tracker.InputProductId ?? "-"} / SCARA {tracker.ScaraProductId ?? "-"} / 인계 {tracker.HandoffProductId ?? "-"} / FANUC {tracker.FanucProductId ?? "-"}";
                if (tracker != null && tracker.IsFaulted) products.text += "\n추적 고장 유지: " + tracker.FaultReason;
                else if (tracker != null && !string.IsNullOrEmpty(tracker.LastAcknowledgedFaultReason)) products.text += "\n최근 추적 명시 해제: " + tracker.LastAcknowledgedFaultReason;
                foreach (var pair in commands)
                {
                    bool allowed = cell.CanExecute(pair.Key, out var reason);
                    pair.Value.SetEnabled(allowed); pair.Value.tooltip = allowed ? "실행 직전 상태/경로를 다시 검사합니다." : reason;
                }
                takeScara.SetEnabled(cell.CanTakeControl(scara, out var scaraReason)); takeScara.tooltip = scaraReason;
                takeFanuc.SetEnabled(cell.CanTakeControl(fanuc, out var fanucReason)); takeFanuc.tooltip = fanucReason;
                release.SetEnabled(cell.CanReleaseControl(out var releaseReason)); release.tooltip = releaseReason;
                mode.text = !playing ? "Play 실행 후 셀과 개별 경로를 조작할 수 있습니다."
                    : !cell.HasControlledSequence ? "인계 없음: 공통 시작은 운전 허가만 바꿉니다. 로봇은 자동 출발하지 않습니다."
                    : cell.ControlledSequence != null && cell.ControlledSequence.State == RobotWaypointSequence.RunState.Completed ? "인계 경로 완료: 시작/재개로 반복하지 않습니다. 다음 개별 시험은 정지 후 권한을 반환하세요."
                    : "인계된 저장 경로 하나를 공통 시작/재개로 이어갑니다.";
                fault.text = cell.IsFaulted ? "고장 유지: " + cell.FaultReason
                    : string.IsNullOrEmpty(cell.LastAcknowledgedFaultReason) ? "" : "최근 명시 해제: " + cell.LastAcknowledgedFaultReason;
                fault.messageType = cell.IsFaulted ? HelpBoxMessageType.Warning : HelpBoxMessageType.Info;
                fault.style.display = string.IsNullOrEmpty(fault.text) ? DisplayStyle.None : DisplayStyle.Flex;
                request.text = requestMessage ?? ""; request.messageType = requestAccepted ? HelpBoxMessageType.Info : HelpBoxMessageType.Warning;
                request.style.display = string.IsNullOrEmpty(request.text) ? DisplayStyle.None : DisplayStyle.Flex;
            }
            Refresh(); root.schedule.Execute(Refresh).Every(100);
            return root;
        }

        private static string RobotText(string id, RobotWaypointSequence sequence)
        {
            if (sequence == null) return id + ": 경로 연결 없음";
            var gripper = sequence.GetComponent<RobotGripper>(); var item = gripper != null ? gripper.HeldItem : null;
            string held = item == null ? "없음" : item.Owner != gripper ? "보유 불일치" : item.ProductId ?? item.name;
            string step = sequence.State == RobotWaypointSequence.RunState.Idle ? "대기"
                : sequence.State == RobotWaypointSequence.RunState.Completed ? "완료" : $"{sequence.StepIndex + 1}/4 {sequence.StepName}";
            string reason = sequence.State == RobotWaypointSequence.RunState.Faulted ? "\n경로 고장 유지: " + sequence.LastError
                : string.IsNullOrEmpty(sequence.LastAcknowledgedFaultReason) ? "" : "\n최근 경로 명시 해제: " + sequence.LastAcknowledgedFaultReason;
            return $"{id}: {sequence.State} · 단계 {step} · 실물 보유 {held}" + reason;
        }
    }
}
