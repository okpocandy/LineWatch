using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    [CustomEditor(typeof(RobotWaypointSequence))]
    public sealed class RobotWaypointSequenceEditor : UnityEditor.Editor
    {
        public override VisualElement CreateInspectorGUI()
        {
            var root = new VisualElement();
            root.Add(new PropertyField(serializedObject.FindProperty("waypoints"), "경유점 데이터"));
            root.Add(new PropertyField(serializedObject.FindProperty("scara"), "SCARA 연결"));
            root.Add(new PropertyField(serializedObject.FindProperty("fanuc"), "FANUC 연결"));
            root.Add(CreateControls((RobotWaypointSequence)target));
            return root;
        }

        internal static VisualElement CreateControls(RobotWaypointSequence sequence)
        {
            var box = new VisualElement { name = "waypoint-controls" };
            box.style.marginTop = box.style.marginBottom = 10;
            var title = new Label("고정 경유점 순차 이동"); title.style.unityFontStyleAndWeight = FontStyle.Bold;
            box.Add(title);
            if (sequence == null) { box.Add(new HelpBox("경유점 운전 연결을 확인하세요.", HelpBoxMessageType.Warning)); return box; }
            var route = new Label(); box.Add(route);
            var data = new ObjectField("경유점 데이터") { name = "waypoint-data", value = sequence.Waypoints, objectType = typeof(RobotWaypointSet), allowSceneObjects = false };
            data.SetEnabled(false); box.Add(data);
            var row = new VisualElement(); row.style.flexDirection = FlexDirection.Row;
            string requestError = null;
            var start = new Button(() => { requestError = sequence.TryStart(out var e) ? null : e; }) { name = "waypoint-start", text = "처음부터 시작" };
            var stop = new Button(() => { sequence.Pause(); requestError = null; }) { name = "waypoint-stop", text = "정지" };
            var resume = new Button(() => { requestError = sequence.TryResume(out var e) ? null : e; }) { name = "waypoint-resume", text = "재개" };
            foreach (var button in new[] { start, stop, resume }) { button.style.flexGrow = 1; button.style.minHeight = 28; row.Add(button); }
            box.Add(row);
            var state = new Label { name = "waypoint-state" }; state.style.whiteSpace = WhiteSpace.Normal; box.Add(state);
            var error = new HelpBox("", HelpBoxMessageType.Warning) { name = "waypoint-error" }; box.Add(error);
            var cellLock = new HelpBox("", HelpBoxMessageType.Info) { name = "waypoint-cell-lock" }; box.Add(cellLock);
            box.Add(new HelpBox("SCARA는 첫 접근까지 관절 이동, 이후 직선 이동입니다. FANUC는 관절 이동이며 TCP 직선·방향 유지를 보장하지 않습니다.", HelpBoxMessageType.Info));
            void Refresh()
            {
                if (sequence == null) return;
                data.SetValueWithoutNotify(sequence.Waypoints);
                bool playing = EditorApplication.isPlaying, clock = !EditorApplication.isPaused && Time.timeScale > 0f;
                bool running = sequence.IsBusy;
                route.text = sequence.IsFanucTransfer ? $"인계 접근 → 집기 → 상승 → {sequence.FanucDestination} 출구 접근 → 놓기 → 상승 → 퇴피"
                    : sequence.IsCellTransfer ? "투입 접근 → 집기 → 상승 → 인계 접근 → 놓기 → 상승 → 퇴피"
                    : sequence.IsPickPlace ? "접근 → 작업/집기 → 상승 → 퇴피/놓기" : "접근 → 작업 → 상승 → 퇴피 · 집기 동작 없음";
                bool paused = sequence.State == RobotWaypointSequence.RunState.Paused || sequence.State == RobotWaypointSequence.RunState.Faulted;
                bool manual = CellRunControllerEditor.ManualAllowed(sequence, out var lockReason);
                var gripper = sequence.GetComponent<RobotGripper>();
                bool holdingCycle = sequence.IsPickPlace && gripper != null && gripper.HeldItem != null;
                start.SetEnabled(playing && clock && sequence.IsReady && !running && manual && !holdingCycle);
                stop.SetEnabled(playing && running);
                resume.SetEnabled(playing && clock && sequence.IsReady && paused && manual);
                cellLock.text = lockReason ?? "";
                cellLock.style.display = playing && !manual ? DisplayStyle.Flex : DisplayStyle.None;
                string status = !playing ? "Play 대기" : sequence.State == RobotWaypointSequence.RunState.Idle ? "대기"
                    : sequence.State == RobotWaypointSequence.RunState.Completed ? sequence.StepCount + "/" + sequence.StepCount + " 완료"
                    : sequence.State == RobotWaypointSequence.RunState.Cancelled ? "명시 종료 · 재개 불가"
                    : sequence.IsBusy ? clock ? "실행 중" : "에디터/시간 일시정지"
                    : sequence.State == RobotWaypointSequence.RunState.Paused ? "정지" : "오류 중단";
                state.text = !playing || sequence.State == RobotWaypointSequence.RunState.Idle || sequence.State == RobotWaypointSequence.RunState.Completed
                    ? status : $"{status} · {sequence.StepIndex + 1}/{sequence.StepCount} {sequence.StepName}";
                error.text = string.IsNullOrEmpty(sequence.LastError) ? requestError ?? "" : sequence.LastError +
                    (string.IsNullOrEmpty(requestError) ? "" : "\n마지막 요청 거부: " + requestError);
                error.style.display = string.IsNullOrEmpty(error.text) ? DisplayStyle.None : DisplayStyle.Flex;
            }
            Refresh(); box.schedule.Execute(Refresh).Every(100);
            return box;
        }
    }
}
