using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    [CustomEditor(typeof(RobotGripperLab))]
    public sealed class RobotGripperLabEditor : UnityEditor.Editor
    {
        public override VisualElement CreateInspectorGUI()
        {
            var root = new VisualElement();
            root.Add(new PropertyField(serializedObject.FindProperty("gripper"), "그리퍼"));
            root.Add(new PropertyField(serializedObject.FindProperty("waypoints"), "경유점"));
            root.Add(new PropertyField(serializedObject.FindProperty("trialPrefab"), "시험 물체"));
            root.Add(CreateControls((RobotGripperLab)target));
            return root;
        }

        internal static VisualElement CreateControls(RobotGripperLab lab)
        {
            var box = new VisualElement { name = "gripper-controls" };
            box.style.marginTop = box.style.marginBottom = 10;
            var title = new Label("집기·놓기 시험"); title.style.unityFontStyleAndWeight = FontStyle.Bold; box.Add(title);
            if (lab == null) { box.Add(new HelpBox("집기 시험 연결을 확인하세요.", HelpBoxMessageType.Warning)); return box; }
            box.Add(new Label("정지 상태 · 접촉 기준 2mm / 0.5° 이내에서 집기"));
            string requestError = null;
            var prepare = new Button(() => { requestError = lab.TryPrepare(out var e) ? null : e; }) { name = "gripper-prepare", text = "시험 물체 준비" };
            var work = new Button(() => { requestError = lab.TryMoveToWork(out var e) ? null : e; }) { name = "gripper-work", text = "작업점 이동" };
            var grip = new Button(() => { requestError = lab.Gripper.TryGrip(lab.TrialItem, out var e) ? null : e; }) { name = "gripper-grip", text = "집기" };
            var lift = new Button(() => { requestError = lab.TryLift(out var e) ? null : e; }) { name = "gripper-lift", text = "상승" };
            var stop = new Button(() => { lab.Gripper.Stop(); requestError = null; }) { name = "gripper-stop", text = "정지", tooltip = "물체 보유를 유지하며 로봇을 정지합니다." };
            var release = new Button(() => { requestError = lab.Gripper.TryRelease(out var e) ? null : e; }) { name = "gripper-release", text = "놓기" };
            var sequence = lab.GetComponent<RobotWaypointSequence>();
            var autoStart = new Button(() => { requestError = lab.TryStartPickPlace(out var e) ? null : e; }) { name = "gripper-cycle-start", text = "자동 운전 시작" };
            var autoStop = new Button(() => { if (sequence != null) sequence.Pause(); requestError = null; }) { name = "gripper-cycle-stop", text = "자동 운전 정지" };
            var autoResume = new Button(() =>
            {
                if (sequence == null) { requestError = "순차 운전 연결을 확인하세요."; return; }
                requestError = sequence.TryResume(out var e) ? null : e;
            }) { name = "gripper-cycle-resume", text = "자동 운전 재개" };
            void Row(params Button[] buttons)
            {
                var row = new VisualElement(); row.style.flexDirection = FlexDirection.Row;
                foreach (var b in buttons) { b.style.flexGrow = 1; b.style.minHeight = 28; row.Add(b); }
                box.Add(row);
            }
            box.Add(new Label("자동 이동·집기: 준비 후 시작 · 퇴피점에서 놓기"));
            Row(prepare, autoStart); Row(autoStop, autoResume);
            var autoState = new Label { name = "gripper-cycle-state" }; box.Add(autoState);
            box.Add(new Label("수동 집기 시험"));
            Row(work, grip, lift); Row(stop, release);
            var item = new ObjectField("시험 대상") { name = "gripper-item", objectType = typeof(GrippableItem), allowSceneObjects = true };
            item.SetEnabled(false); box.Add(item);
            var state = new Label { name = "gripper-state" }; box.Add(state);
            var alignment = new Label { name = "gripper-alignment" }; box.Add(alignment);
            var error = new HelpBox("", HelpBoxMessageType.Warning) { name = "gripper-error" }; box.Add(error);
            box.Add(new HelpBox("자동: 준비 → 시작 → 작업점 집기 → 상승/퇴피 → 놓기. 정지는 보유를 유지하고 재개는 남은 구간만 수행합니다. 놓은 물체는 그 위치에 유지됩니다.", HelpBoxMessageType.Info));
            void Refresh()
            {
                if (lab == null || lab.Gripper == null) return;
                var g = lab.Gripper; var target = lab.TrialItem;
                bool playing = EditorApplication.isPlaying, clock = !EditorApplication.isPaused && Time.timeScale > 0f;
                bool ready = playing && clock && lab.IsReady && g.IsStopped;
                bool busy = sequence != null && sequence.IsBusy;
                autoStart.SetEnabled(ready && sequence != null && !busy && target != null && target.Owner == null && g.HeldItem == null);
                autoStop.SetEnabled(playing && busy);
                autoResume.SetEnabled(ready && sequence != null && sequence.IsPickPlace &&
                    (sequence.State == RobotWaypointSequence.RunState.Paused || sequence.State == RobotWaypointSequence.RunState.Faulted));
                autoState.text = !playing ? "Play 대기" : sequence == null || !sequence.IsPickPlace ? "자동 집기 운전 대기"
                    : sequence.State == RobotWaypointSequence.RunState.Completed ? "자동 운전 완료 · 물체 놓기 완료"
                    : $"자동 운전 {(sequence.State == RobotWaypointSequence.RunState.Paused ? "정지" : sequence.State == RobotWaypointSequence.RunState.Faulted ? "오류 중단" : "실행 중")} · {sequence.StepName}";
                item.SetValueWithoutNotify(target);
                prepare.SetEnabled(ready && g.HeldItem == null && (target == null || target.Owner == null));
                work.SetEnabled(ready); lift.SetEnabled(ready);
                grip.SetEnabled(ready && target != null && g.HeldItem == null);
                release.SetEnabled(ready && g.HeldItem != null);
                stop.SetEnabled(playing);
                state.text = !playing ? "Play 대기" : g.HeldItem != null ? "보유: " + g.HeldItem.name : "보유: 없음";
                alignment.text = target != null && target.IsReady && g.Contact != null
                    ? $"기준 오차: {Vector3.Distance(g.Contact.position, target.GripPose.position) * 1000:F3}mm / {RobotGripper.RotationError(g.Contact.rotation, target.GripPose.rotation):F3}°" : "시험 물체를 준비하세요.";
                error.text = (sequence != null && sequence.IsPickPlace && !string.IsNullOrEmpty(sequence.LastError) ? sequence.LastError + "\n" : "") +
                    (string.IsNullOrEmpty(g.LastError) ? "" : "마지막 보유 중단: " + g.LastError + "\n") + (requestError ?? "");
                error.style.display = string.IsNullOrEmpty(error.text) ? DisplayStyle.None : DisplayStyle.Flex;
            }
            Refresh(); box.schedule.Execute(Refresh).Every(100);
            return box;
        }
    }
}
