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
            var automaticState = new Label { name = "cell-run-automatic-transfer" };
            var scaraState = new Label { name = "cell-run-scara" };
            var fanucState = new Label { name = "cell-run-fanuc" };
            var products = new Label { name = "cell-run-products" };
            foreach (var label in new[] { state, owner, automaticState, scaraState, fanucState, products })
            { label.style.whiteSpace = WhiteSpace.Normal; label.style.marginTop = 6; root.Add(label); }
            var mode = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-run-mode" }; root.Add(mode);
            var fault = new HelpBox("", HelpBoxMessageType.Warning) { name = "cell-run-fault" }; root.Add(fault);
            var commands = new Dictionary<CellRunCommand, Button>();
            string requestMessage = null; bool requestAccepted = false;
            ProductRecord transferPreview = null;
            var prepareAutomaticTransfer = new Button(() =>
            {
                if (cell == null) { requestAccepted = false; requestMessage = "셀 운전 관리자가 없습니다."; return; }
                requestAccepted = cell.TryPrepareAutomaticTransfer(transferPreview, out var error);
                requestMessage = requestAccepted ? "자동 단품 준비 완료 · 공통 시작으로 두 로봇을 순서대로 운전하세요." : error;
            }) { name = "cell-run-prepare-automatic-transfer", text = "단품 운전 준비" };
            var prepareContinuousTransfer = new Button(() =>
            {
                if (cell == null) { requestAccepted = false; requestMessage = "셀 운전 관리자가 없습니다."; return; }
                requestAccepted = cell.TryPrepareContinuousTransfer(transferPreview, out var error);
                requestMessage = requestAccepted ? "연속 운전 준비 완료 · 공통 시작으로 등록된 대기 상품까지 운전하세요." : error;
            }) { name = "cell-run-prepare-continuous-transfer", text = "연속 운전 준비" };
            ProductRecord fanucTransferPreview = null;
            var fanucTransferTarget = new Label { name = "cell-run-fanuc-transfer-target" };
            fanucTransferTarget.style.whiteSpace = WhiteSpace.Normal;
            var prepareFanucTransfer = new Button(() =>
            {
                if (cell == null) { requestAccepted = false; requestMessage = "셀 운전 관리자가 없습니다."; return; }
                var preview = fanucTransferPreview;
                if (preview == null) { requestAccepted = false; requestMessage = "표시된 인계 상품이 없습니다."; return; }
                requestAccepted = cell.TryPrepareFanucTransfer(preview.Destination, preview, out var error);
                requestMessage = requestAccepted ? "FANUC 운반 준비 완료 · 공통 시작을 눌러 운전하세요." : error;
            }) { name = "cell-run-prepare-fanuc-transfer", text = "FANUC 인계 상품 운반 준비" };
            var transferTarget = new Label { name = "cell-run-transfer-target" };
            transferTarget.style.whiteSpace = WhiteSpace.Normal;
            var prepareTransfer = new Button(() =>
            {
                if (cell == null) { requestAccepted = false; requestMessage = "셀 운전 관리자가 없습니다."; return; }
                requestAccepted = cell.TryPrepareScaraTransfer(transferPreview, out var error);
                requestMessage = requestAccepted ? "SCARA 운반 준비 완료 · 공통 시작을 눌러 운전하세요." : error;
            }) { name = "cell-run-prepare-scara-transfer", text = "SCARA 투입 상품 운반 준비" };
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
                requestMessage = requestAccepted ? cell.IsFaulted ? "복구 인계 수락 · 고장 해제 후 별도 공통 시작을 요청하세요."
                    : text + " 수락 · 공통 시작으로 경로를 이어가세요." : error;
            }) { name = name, text = text };
            var takeScara = Take(scara, "cell-run-take-scara", "SCARA 경로 인계");
            var takeFanuc = Take(fanuc, "cell-run-take-fanuc", "FANUC 경로 인계");
            var release = new Button(() =>
            {
                if (cell == null) { requestAccepted = false; requestMessage = "셀 운전 관리자가 없습니다."; return; }
                requestAccepted = cell.TryReleaseControl(out var error);
                requestMessage = requestAccepted ? "권한 반환 수락 · 저장 경로와 보유는 유지됩니다." : error;
            }) { name = "cell-run-return", text = "권한 반환 · 새 준비" };
            void Row(VisualElement parent, params Button[] buttons)
            {
                var row = new VisualElement(); row.style.flexDirection = FlexDirection.Row; row.style.flexWrap = Wrap.Wrap;
                foreach (var b in buttons)
                { b.style.flexGrow = 1; b.style.flexBasis = 0; b.style.minWidth = 100; b.style.minHeight = 28; b.style.marginTop = 5; row.Add(b); }
                parent.Add(row);
            }
            root.Add(transferTarget); Row(root, prepareAutomaticTransfer, prepareContinuousTransfer);
            root.Add(new HelpBox("단품: 투입 1개. 연속: 투입 상품과 등록된 대기열. 준비 후 시작을 누르면 공급·운반하고 대기열이 비면 정지합니다. 대기 상품이 없으면 연속도 1개만 운반합니다.", HelpBoxMessageType.Info));
            Row(root, start, pause, resume); Row(root, stop, reset); Row(root, release);
            var request = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-run-request" }; root.Add(request);
            var individual = new Foldout { name = "cell-run-individual-tests", text = "개별 경로 시험 · 고급", value = false };
            individual.Add(root.Q<ObjectField>("cell-run-controller")); individual.Add(owner);
            individual.Add(new HelpBox("개별 로봇의 운반/저장 경로만 시험할 때 사용합니다. 자동 단품 운전에는 필요하지 않습니다.", HelpBoxMessageType.Info));
            Row(individual, prepareTransfer); individual.Add(fanucTransferTarget); Row(individual, prepareFanucTransfer);
            Row(individual, takeScara, takeFanuc); root.Add(individual);
            var recovery = new Foldout { name = "cell-run-recovery-tools", text = "복구·초기화 · 고급", value = false };
            recovery.Add(new HelpBox("실물/ID/보유 확인 후 별도 고장 해제를 요청하세요. 손실 확정·대기 복귀·경로 종료·시나리오 초기화는 아래에서 명시적으로 조작합니다. 초기화에는 모든 상품 종료가 필요합니다.", HelpBoxMessageType.Info));
            recovery.Add(CreateLossControls(cell)); recovery.Add(CreateCancellationControls(cell, scara, fanuc));
            recovery.Add(CreateQueueRecoveryControls(cell)); recovery.Add(CreateScenarioControls(cell)); root.Add(recovery);
            void Refresh()
            {
                if (cell == null)
                {
                    foreach (var b in commands.Values.Concat(new[] { prepareAutomaticTransfer, prepareContinuousTransfer, prepareTransfer, prepareFanucTransfer, takeScara, takeFanuc, release })) b.SetEnabled(false);
                    transferPreview = null; transferTarget.text = "투입 상품 확인 불가";
                    automaticState.text = "자동 단품: 관리자 없음 · 상태 확인 불가";
                    fanucTransferPreview = null; fanucTransferTarget.text = "인계 상품 확인 불가";
                    state.text = "셀 상태: 관리자 없음 · 조작 차단"; owner.text = "명령 권한: 확인 불가";
                    scaraState.text = "SCARA: 관리자 없음 · 상태 확인 불가"; fanucState.text = "FANUC: 관리자 없음 · 상태 확인 불가";
                    products.text = "상품 추적: 별도 관리자와 실제 보유를 확인하세요.";
                    mode.text = "관리자 소실 후 개별 명령을 허용하지 않습니다.";
                    fault.text = "셀 운전 관리자가 사라졌습니다. 개별 조작으로 자동 전환하지 않습니다."; fault.style.display = DisplayStyle.Flex;
                    return;
                }
                bool playing = Application.isPlaying;
                state.text = playing ? "셀 상태: " + cell.State : "셀 상태: Play 대기";
                string automaticPhase = cell.AutomaticPhase == CellRunController.AutomaticTransferPhase.Scara ? "SCARA 운반/퇴피"
                    : cell.AutomaticPhase == CellRunController.AutomaticTransferPhase.Fanuc ? "FANUC 배출"
                    : cell.AutomaticPhase == CellRunController.AutomaticTransferPhase.WaitingForInput ? "투입 공급/정렬" : "완료";
                automaticState.text = !playing ? "자동 단품: Play 대기"
                    : cell.AutomaticPhase == CellRunController.AutomaticTransferPhase.None ? "자동 단품: 미준비"
                    : (cell.IsAutomaticContinuous ? "연속 운전: " : "자동 단품: ") + cell.AutomaticProductId + " · 목적지 " + cell.AutomaticDestination + " · " + automaticPhase;
                owner.text = "명령 권한: " + (!playing ? "Play 대기" : !cell.HasControlledSequence ? "개별 시험"
                    : cell.ControlledSequence == null ? "공통 제어 · 경로 연결 없음"
                    : cell.ControlledSequence == scara ? layout.ScaraId + " 경로 · 공통 제어"
                    : cell.ControlledSequence == fanuc ? layout.FanucId + " 경로 · 공통 제어" : "공통 제어 · 경로 연결 없음");
                scaraState.text = RobotText(layout != null ? layout.ScaraId : "SCARA", scara);
                fanucState.text = RobotText(layout != null ? layout.FanucId : "FANUC", fanuc);
                var tracker = cell.GetComponent<CellProductTracker>();
                transferPreview = null;
                if (playing && tracker != null) tracker.TryGet(tracker.InputProductId, out transferPreview);
                transferTarget.text = !playing ? "운전 준비 · Play 대기"
                    : transferPreview == null ? "현재 투입 상품 없음"
                    : "투입 상품: " + transferPreview.Id + " · 목적지 " + transferPreview.Destination;
                bool prepareAllowed = cell.CanPrepareScaraTransfer(out var prepareReason);
                prepareTransfer.SetEnabled(prepareAllowed); prepareTransfer.tooltip = prepareAllowed
                    ? "표시된 상품 기록·전체 경로를 실행 직전 다시 확인합니다. 준비 후 별도 시작이 필요합니다." : prepareReason;
                bool automaticAllowed = cell.CanPrepareAutomaticTransfer(out var automaticReason);
                prepareAutomaticTransfer.SetEnabled(automaticAllowed); prepareAutomaticTransfer.tooltip = automaticAllowed
                    ? "표시된 한 상품만 준비합니다. 준비 후 별도 공통 시작이 필요합니다." : automaticReason;
                bool continuousAllowed = cell.CanPrepareContinuousTransfer(out var continuousReason);
                prepareContinuousTransfer.SetEnabled(continuousAllowed); prepareContinuousTransfer.tooltip = continuousAllowed
                    ? "표시된 투입 상품부터 등록된 FIFO를 운전합니다. 준비 후 별도 시작이 필요합니다." : continuousReason;
                fanucTransferPreview = null;
                if (playing && tracker != null) tracker.TryGet(tracker.HandoffProductId, out fanucTransferPreview);
                fanucTransferTarget.text = !playing ? "FANUC 상품 운반 · Play 대기"
                    : fanucTransferPreview == null ? "인계 상품 없음 · SCARA 운반과 퇴피를 먼저 완료하세요."
                    : "인계 상품: " + fanucTransferPreview.Id + " · 목적지 " + fanucTransferPreview.Destination + " → " + fanucTransferPreview.Destination + " 출구";
                string fanucPrepareReason = "인계 상품이 필요합니다.";
                bool fanucPrepareAllowed = fanucTransferPreview != null && cell.CanPrepareFanucTransfer(fanucTransferPreview.Destination, out fanucPrepareReason);
                prepareFanucTransfer.SetEnabled(fanucPrepareAllowed); prepareFanucTransfer.tooltip = fanucPrepareAllowed
                    ? "표시된 상품 기록·목적지·전체 경로를 다시 확인합니다. 준비 후 별도 시작이 필요합니다." : fanucPrepareReason;
                products.text = tracker == null ? "상품 추적: 연결 없음" : $"상품 등록 기록: {tracker.Count} · 대기 {tracker.WaitingSnapshot().Count}\n투입 {tracker.InputProductId ?? "-"} / SCARA {tracker.ScaraProductId ?? "-"} / 인계 {tracker.HandoffProductId ?? "-"} / FANUC {tracker.FanucProductId ?? "-"}";
                if (tracker != null && tracker.IsFaulted) products.text += "\n추적 고장 유지: " + tracker.FaultReason;
                else if (tracker != null && !string.IsNullOrEmpty(tracker.LastAcknowledgedFaultReason)) products.text += "\n최근 추적 명시 해제: " + tracker.LastAcknowledgedFaultReason;
                if (tracker != null)
                {
                    var current = tracker.CurrentScenarioSnapshot();
                    products.text += $"\n시나리오 #{tracker.ScenarioNumber}: 등록 {current.Length} · 완료 {current.Count(r => r.Location == ProductLocation.OutputA || r.Location == ProductLocation.OutputB)} · 제거 {current.Count(r => r.Location == ProductLocation.Removed)} · 손실 {current.Count(r => r.Location == ProductLocation.Lost)} · 초기화 이력 {tracker.ScenarioSnapshot().Length}";
                    var recoveries = tracker.QueueRecoverySnapshot();
                    if (recoveries.Length > 0) products.text += "\n명시 대기 복귀 " + recoveries.Length + ": " + recoveries[recoveries.Length - 1].Previous.Id + " · " + recoveries[recoveries.Length - 1].Reason;
                    var losses = tracker.Snapshot().Where(r => r.Location == ProductLocation.Lost).ToArray();
                    if (losses.Length > 0) products.text += "\n명시 손실 확정 " + losses.Length + ": " +
                        string.Join(" / ", losses.Select(r => r.Id + " · 이전 " + r.LostFrom + " · " + r.LossReason));
                }
                foreach (var pair in commands)
                {
                    bool allowed = cell.CanExecute(pair.Key, out var reason);
                    pair.Value.SetEnabled(allowed); pair.Value.tooltip = allowed ? "실행 직전 상태/경로를 다시 검사합니다." : reason;
                }
                takeScara.SetEnabled(cell.CanTakeControl(scara, out var scaraReason)); takeScara.tooltip = scaraReason;
                takeFanuc.SetEnabled(cell.CanTakeControl(fanuc, out var fanucReason)); takeFanuc.tooltip = fanucReason;
                takeScara.text = cell.IsFaulted ? "SCARA 고장 복구 인계" : "SCARA 경로 인계";
                takeFanuc.text = cell.IsFaulted ? "FANUC 고장 복구 인계" : "FANUC 경로 인계";
                release.SetEnabled(cell.CanReleaseControl(out var releaseReason)); release.tooltip = releaseReason;
                mode.text = !playing ? "Play 후 단품 또는 연속 준비 → 시작 순서로 운전하세요."
                    : cell.IsFaulted ? "고장을 확인한 뒤 별도 해제하세요. 상세 조작은 복구·초기화(고급)에 있습니다."
                    : cell.AutomaticPhase == CellRunController.AutomaticTransferPhase.Completed ? "운전 완료 · 권한 반환 후 새 상품을 준비하세요."
                    : cell.AutomaticPhase != CellRunController.AutomaticTransferPhase.None ? "공통 조작으로 운전·일시정지·재개·정지합니다."
                    : !cell.HasControlledSequence ? "단품 또는 연속 준비 후 시작하세요. 개별 시험은 고급 영역에 있습니다."
                    : "개별 경로 인계 상태 · 공통 시작/재개로 이어갑니다.";
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

        private static VisualElement CreateLossControls(CellRunController cell)
        {
            var panel = new Foldout { name = "cell-loss-panel", text = "손실 확정 · 삭제 원본만", value = false };
            var tracker = cell.GetComponent<CellProductTracker>();
            const string placeholder = "대상을 선택하세요";
            ProductRecord selected = null;
            bool updatingChoices = false, accepted = false;
            string message = null;
            var target = new DropdownField("상품 ID", new List<string> { placeholder }, 0) { name = "cell-loss-target" };
            var reason = new TextField("손실 확정 사유") { name = "cell-loss-reason", multiline = true };
            reason.style.minHeight = 48; reason.style.maxHeight = 100;
            var selection = new Label { name = "cell-loss-selection" }; selection.style.whiteSpace = WhiteSpace.Normal;
            var eligibility = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-loss-eligibility" };
            var result = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-loss-request" };
            panel.Add(new HelpBox("등록된 원래 GameObject가 삭제된 상품만 손실로 확정합니다. ID/사유/선택 당시 위치를 확인하세요. 대상 변경 시 사유는 비워집니다. Lost 이력과 ID는 남고 고장은 유지됩니다. 실제 보유나 미완료 경로가 있으면 거부합니다.", HelpBoxMessageType.Info));
            panel.Add(target); panel.Add(reason); panel.Add(selection); panel.Add(eligibility);
            Button confirm = null;
            confirm = new Button(() =>
            {
                accepted = false;
                if (cell == null || tracker == null) message = "셀/추적기 연결을 확인하세요.";
                else if (selected == null) message = "손실 확정 대상을 선택하세요.";
                else
                {
                    string id = selected.Id;
                    // Use the location captured on explicit selection. Never replace the
                    // operator's request with a newer record during scheduled refreshes.
                    accepted = tracker.TryConfirmDestroyedLoss(selected, reason.value, out var error);
                    message = accepted ? id + " 손실 확정 기록 · 고장 해제는 별도 요청입니다." : error;
                    if (accepted)
                    {
                        selected = null;
                        target.SetValueWithoutNotify(placeholder);
                        reason.SetValueWithoutNotify("");
                    }
                }
                Refresh();
            }) { name = "cell-loss-confirm", text = "선택 상품 손실 확정 · 기록 유지" };
            confirm.style.minHeight = 28; panel.Add(confirm); panel.Add(result);
            target.RegisterValueChangedCallback(evt =>
            {
                if (updatingChoices) return;
                selected = null;
                reason.SetValueWithoutNotify("");
                if (tracker != null && evt.newValue.StartsWith("ID ") && tracker.TryGet(evt.newValue.Substring(3), out var record)) selected = record;
                Refresh();
            });
            reason.RegisterValueChangedCallback(_ => Refresh());
            void Refresh()
            {
                bool ready = Application.isPlaying && cell != null && cell.isActiveAndEnabled && tracker != null && tracker.isActiveAndEnabled;
                target.SetEnabled(ready); reason.SetEnabled(ready);
                var choices = new List<string> { placeholder };
                if (ready)
                    choices.AddRange(tracker.Snapshot().Where(r => r.Location == ProductLocation.Waiting || r.Location == ProductLocation.Input ||
                        r.Location == ProductLocation.Scara || r.Location == ProductLocation.Handoff || r.Location == ProductLocation.Fanuc).Select(r => "ID " + r.Id));
                string selectedChoice = selected == null ? placeholder : "ID " + selected.Id;
                // Retain a now-terminal/stale selection visibly instead of silently
                // switching to another ID. The API rejects its captured old location.
                if (selected != null && !choices.Contains(selectedChoice)) choices.Add(selectedChoice);
                if (!target.choices.SequenceEqual(choices))
                {
                    updatingChoices = true;
                    try { target.choices = choices; target.SetValueWithoutNotify(selectedChoice); }
                    finally { updatingChoices = false; }
                }
                selection.text = selected == null ? "선택된 상품 없음 · 대상은 자동 선택하지 않습니다."
                    : selected.Id + " · 목적지 " + selected.Destination + " · 선택 당시 " + selected.Location + " / 현재 " +
                        (tracker != null && tracker.TryGet(selected.Id, out var current) ? current.Location.ToString() : "확인 불가");
                string why = !ready ? "활성 LineWatch Play에서 대상을 선택하세요." : selected == null ? "손실 확정 대상을 선택하세요." : null;
                bool canConfirm = ready && selected != null && tracker.CanConfirmDestroyedLoss(selected, reason.value, out why);
                confirm.SetEnabled(canConfirm);
                confirm.tooltip = canConfirm ? "실행 직전 삭제 증거·보유·경로와 선택 당시 위치를 다시 검사합니다." : why;
                eligibility.text = canConfirm ? "삭제 증거와 운전 조건 확인 · ID/사유를 검토한 뒤 명시적으로 확정하세요." : why;
                result.text = message ?? ""; result.messageType = accepted ? HelpBoxMessageType.Info : HelpBoxMessageType.Warning;
                result.style.display = string.IsNullOrEmpty(message) ? DisplayStyle.None : DisplayStyle.Flex;
            }
            Refresh(); panel.schedule.Execute(Refresh).Every(100);
            return panel;
        }

        private static VisualElement CreateCancellationControls(CellRunController cell, RobotWaypointSequence scara, RobotWaypointSequence fanuc)
        {
            const string placeholder = "대상을 선택하세요";
            var panel = new Foldout { name = "cell-cancel-panel", text = "경로 명시 종료 · 상품/고장 유지", value = false };
            var target = new DropdownField("종료할 경로", new List<string> { placeholder, "SCARA", "FANUC" }, 0) { name = "cell-cancel-target" };
            var reason = new TextField("경로 종료 사유") { name = "cell-cancel-reason", multiline = true };
            reason.style.minHeight = 48; reason.style.maxHeight = 100;
            var selection = new Label { name = "cell-cancel-selection" }; selection.style.whiteSpace = WhiteSpace.Normal;
            var eligibility = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-cancel-eligibility" };
            var result = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-cancel-request" };
            RobotWaypointSequence selected = null; uint revision = 0; string message = null; bool accepted = false;
            panel.Add(new HelpBox("Faulted 상태에서 정지된 경로만 종료합니다. 저장 단계·집기/놓기·종료 사유는 이력으로 남고 상품·보유·고장은 유지됩니다. 이 경로는 재개할 수 없습니다. 손실 확정/고장 해제/새 시작은 별도입니다. 대상 변경 시 사유가 비워집니다.", HelpBoxMessageType.Info));
            panel.Add(target); panel.Add(reason); panel.Add(selection); panel.Add(eligibility);
            Button confirm = null;
            confirm = new Button(() =>
            {
                accepted = false;
                if (cell == null || selected == null) message = "종료할 경로를 선택하세요.";
                else
                {
                    accepted = cell.TryCancelProgram(selected, revision, reason.value, out var error);
                    message = accepted ? target.value + " 경로 종료 기록 · 상품/고장은 유지됩니다." : error;
                    if (accepted) { selected = null; target.SetValueWithoutNotify(placeholder); reason.SetValueWithoutNotify(""); }
                }
                Refresh();
            }) { name = "cell-cancel-confirm", text = "선택 경로 명시 종료 · 재개 불가" };
            confirm.style.minHeight = 28; panel.Add(confirm); panel.Add(result);
            target.RegisterValueChangedCallback(evt =>
            {
                selected = evt.newValue == "SCARA" ? scara : evt.newValue == "FANUC" ? fanuc : null;
                revision = selected != null ? selected.ProgramRevision : 0;
                reason.SetValueWithoutNotify(""); Refresh();
            });
            reason.RegisterValueChangedCallback(_ => Refresh());
            void Refresh()
            {
                bool ready = Application.isPlaying && cell != null && cell.isActiveAndEnabled;
                target.SetEnabled(ready); reason.SetEnabled(ready);
                selection.text = selected == null ? "선택된 경로 없음 · 자동 선택하지 않습니다."
                    : target.value + " · 선택 경로 #" + revision + " / 현재 #" + selected.ProgramRevision + " · " + selected.State +
                        " · 단계 " + (selected.StepIndex + 1) + " · 집기 " + selected.GripCount + " / 놓기 " + selected.ReleaseCount;
                string why = !ready ? "활성 LineWatch Play에서 대상을 선택하세요." : selected == null ? "종료할 경로를 선택하세요." : null;
                bool allowed = ready && selected != null && cell.CanCancelProgram(selected, revision, reason.value, out why);
                confirm.SetEnabled(allowed); confirm.tooltip = allowed ? "실행 직전 경로 번호·정지·고장·사유를 다시 검사합니다." : why;
                eligibility.text = allowed ? "저장 경로와 사유를 검토한 뒤 명시적으로 종료하세요." : why;
                result.text = message ?? ""; result.messageType = accepted ? HelpBoxMessageType.Info : HelpBoxMessageType.Warning;
                result.style.display = string.IsNullOrEmpty(message) ? DisplayStyle.None : DisplayStyle.Flex;
            }
            Refresh(); panel.schedule.Execute(Refresh).Every(100); return panel;
        }

        private static VisualElement CreateQueueRecoveryControls(CellRunController cell)
        {
            const string placeholder = "대상을 선택하세요";
            var tracker = cell.GetComponent<CellProductTracker>(); ProductRecord selected = null;
            var panel = new Foldout { name = "cell-requeue-panel", text = "원래 실물 대기 복귀 · 보유 없음", value = false };
            var target = new DropdownField("상품 ID", new List<string> { placeholder }, 0) { name = "cell-requeue-target" };
            var reason = new TextField("대기 복귀 사유") { name = "cell-requeue-reason", multiline = true };
            reason.style.minHeight = 48; reason.style.maxHeight = 100;
            var selection = new Label { name = "cell-requeue-selection" }; selection.style.whiteSpace = WhiteSpace.Normal;
            var eligibility = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-requeue-eligibility" };
            var result = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-requeue-request" };
            string message = null; bool accepted = false, refreshing = false;
            panel.Add(new HelpBox("원래 상품이 활성·미보유 상태이고 두 집게/저장 경로가 비었을 때만 대기열 끝으로 복귀합니다. 물체 위치·Owner는 바꾸지 않습니다. 다른 상품의 FIFO와 고장은 유지합니다. 투입 위치 정렬·고장 해제·재투입은 별도입니다.", HelpBoxMessageType.Info));
            panel.Add(target); panel.Add(reason); panel.Add(selection); panel.Add(eligibility);
            Button confirm = null;
            confirm = new Button(() =>
            {
                accepted = false;
                if (cell == null || tracker == null || selected == null) message = "복귀할 상품을 선택하세요.";
                else
                {
                    string id = selected.Id; accepted = tracker.TryReturnToQueue(selected, reason.value, out var error);
                    message = accepted ? id + " 대기열 끝 복귀 기록 · 고장은 유지됩니다." : error;
                    if (accepted) { selected = null; target.SetValueWithoutNotify(placeholder); reason.SetValueWithoutNotify(""); }
                }
                Refresh();
            }) { name = "cell-requeue-confirm", text = "원래 상품을 대기열 끝으로 복귀" };
            confirm.style.minHeight = 28; panel.Add(confirm); panel.Add(result);
            target.RegisterValueChangedCallback(evt =>
            {
                if (refreshing) return;
                selected = null; reason.SetValueWithoutNotify("");
                if (tracker != null && evt.newValue.StartsWith("ID ") && tracker.TryGet(evt.newValue.Substring(3), out var record)) selected = record;
                Refresh();
            });
            reason.RegisterValueChangedCallback(_ => Refresh());
            void Refresh()
            {
                bool ready = Application.isPlaying && cell != null && cell.isActiveAndEnabled && tracker != null && tracker.isActiveAndEnabled;
                target.SetEnabled(ready); reason.SetEnabled(ready);
                var choices = new List<string> { placeholder };
                if (tracker != null) choices.AddRange(tracker.Snapshot().Where(r => r.Location == ProductLocation.Input || r.Location == ProductLocation.Scara || r.Location == ProductLocation.Handoff || r.Location == ProductLocation.Fanuc).Select(r => "ID " + r.Id));
                string choice = selected == null ? placeholder : "ID " + selected.Id;
                if (selected != null && !choices.Contains(choice)) choices.Add(choice);
                if (!target.choices.SequenceEqual(choices))
                {
                    refreshing = true;
                    try { target.choices = choices; target.SetValueWithoutNotify(choice); }
                    finally { refreshing = false; }
                }
                selection.text = selected == null ? "선택된 상품 없음 · 자동 선택하지 않습니다."
                    : selected.Id + " · 선택 당시 " + selected.Location + " / 현재 " + (tracker != null && tracker.TryGet(selected.Id, out var record) ? record.Location.ToString() : "확인 불가");
                string why = !ready ? "활성 LineWatch Play에서 대상을 선택하세요." : selected == null ? "복귀할 상품을 선택하세요." : null;
                bool allowed = ready && selected != null && tracker.CanReturnToQueue(selected, reason.value, out why);
                confirm.SetEnabled(allowed); confirm.tooltip = why;
                eligibility.text = allowed ? "원래 실물·ID·미보유와 선택 당시 기록 확인 · 사유를 검토하세요." : why;
                result.text = message ?? ""; result.messageType = accepted ? HelpBoxMessageType.Info : HelpBoxMessageType.Warning;
                result.style.display = string.IsNullOrEmpty(message) ? DisplayStyle.None : DisplayStyle.Flex;
            }
            Refresh(); panel.schedule.Execute(Refresh).Every(100); return panel;
        }

        private static VisualElement CreateScenarioControls(CellRunController cell)
        {
            var tracker = cell.GetComponent<CellProductTracker>(); int expectedScenario = 0;
            var panel = new Foldout { name = "cell-scenario-panel", text = "시나리오 초기화 · 누적 이력 보존", value = false };
            var reason = new TextField("초기화 사유") { name = "cell-scenario-reason" };
            var eligibility = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-scenario-eligibility" };
            var result = new HelpBox("", HelpBoxMessageType.Info) { name = "cell-scenario-request" };
            bool accepted = false; string message = null;
            panel.Add(new HelpBox("모든 상품의 완료/제거/손실 확정과 별도 고장 해제 후 셀 Stopped·빈 집게·미완료 경로 없음이 필요합니다. 이번 집계는 0, 경로는 Idle, 명령 권한은 반환합니다. 누적 상품/복구/행동 이력·실물·관절 위치는 보존합니다. 새 샘플 생성/자동 시작은 없습니다. 사유 입력 시 시나리오 번호를 고정합니다.", HelpBoxMessageType.Info));
            panel.Add(reason); panel.Add(eligibility);
            Button confirm = null;
            confirm = new Button(() =>
            {
                accepted = false;
                if (cell == null) message = "셀 연결을 확인하세요.";
                else
                {
                    accepted = cell.TryResetScenario(expectedScenario, reason.value, out var error);
                    message = accepted ? "시나리오 초기화 완료 · 새 상품 등록/시작은 별도입니다." : error;
                    if (accepted) { reason.SetValueWithoutNotify(""); expectedScenario = 0; }
                }
                Refresh();
            }) { name = "cell-scenario-confirm", text = "이번 시나리오 초기화 · 누적 이력 보존" };
            confirm.style.minHeight = 28; panel.Add(confirm); panel.Add(result);
            reason.RegisterValueChangedCallback(_ => { expectedScenario = tracker != null ? tracker.ScenarioNumber : 0; Refresh(); });
            void Refresh()
            {
                bool ready = Application.isPlaying && cell != null && cell.isActiveAndEnabled && tracker != null && tracker.isActiveAndEnabled;
                reason.SetEnabled(ready);
                bool hasReason = !string.IsNullOrWhiteSpace(reason.value);
                string why = !ready ? "활성 LineWatch Play가 필요합니다." : !hasReason ? "초기화 사유를 입력하세요." : null;
                bool allowed = ready && hasReason && cell.CanResetScenario(expectedScenario, reason.value, out why);
                confirm.SetEnabled(allowed); confirm.tooltip = why;
                eligibility.text = allowed ? "대상 시나리오 #" + expectedScenario + " · 초기화 영향을 확인한 뒤 실행하세요." : why;
                result.text = message ?? ""; result.messageType = accepted ? HelpBoxMessageType.Info : HelpBoxMessageType.Warning;
                result.style.display = string.IsNullOrEmpty(message) ? DisplayStyle.None : DisplayStyle.Flex;
            }
            Refresh(); panel.schedule.Execute(Refresh).Every(100); return panel;
        }

        private static string RobotText(string id, RobotWaypointSequence sequence)
        {
            if (sequence == null) return id + ": 경로 연결 없음";
            var gripper = sequence.GetComponent<RobotGripper>(); var item = gripper != null ? gripper.HeldItem : null;
            string held = item == null ? "없음" : item.Owner != gripper ? "보유 불일치" : item.ProductId ?? item.name;
            string step = sequence.State == RobotWaypointSequence.RunState.Idle ? "대기"
                : sequence.State == RobotWaypointSequence.RunState.Completed ? "완료"
                : sequence.State == RobotWaypointSequence.RunState.Cancelled ? "명시 종료 · 재개 불가" : $"{sequence.StepIndex + 1}/{sequence.StepCount} {sequence.StepName}";
            string reason = sequence.State == RobotWaypointSequence.RunState.Faulted ? "\n경로 고장 유지: " + sequence.LastError
                : string.IsNullOrEmpty(sequence.LastAcknowledgedFaultReason) ? "" : "\n최근 경로 명시 해제: " + sequence.LastAcknowledgedFaultReason;
            var cancellations = sequence.CancellationSnapshot();
            if (cancellations.Length > 0) reason += $"\n경로 명시 종료 이력 {cancellations.Length}: {cancellations[cancellations.Length - 1].Reason}";
            return $"{id}: {sequence.State} · 단계 {step} · 실물 보유 {held}" + reason;
        }
    }
}
