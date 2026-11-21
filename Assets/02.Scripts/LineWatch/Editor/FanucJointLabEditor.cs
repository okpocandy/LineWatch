using System.Collections.Generic;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    [CustomEditor(typeof(FanucJointLab))]
    public sealed class FanucJointLabEditor : UnityEditor.Editor
    {
        [MenuItem("LineWatch/개별 시험(고급)/FANUC 조작")]
        private static void OpenLab()
        {
            var window = EditorWindow.GetWindow<FanucJointLabWindow>("FANUC 조작 테스트");
            window.minSize = new Vector2(540f, 700f);
            window.position = new Rect(100f, 80f, 580f, 840f);
            window.Show();
            window.RefreshLab();
            window.Focus();
        }

        public override VisualElement CreateInspectorGUI()
        {
            return CreateControls((FanucJointLab)target, serializedObject);
        }

        internal static VisualElement CreateControls(FanucJointLab lab, SerializedObject settingsObject)
        {
            var root = new VisualElement();
            root.Add(RobotGripperLabEditor.CreateControls(lab.GetComponent<RobotGripperLab>()));
            root.Add(RobotWaypointSequenceEditor.CreateControls(lab.GetComponent<RobotWaypointSequence>()));
            var title = new Label("FANUC 관절 조작 테스트");
            title.style.unityFontStyleAndWeight = FontStyle.Bold;
            title.style.fontSize = 16;
            title.style.marginTop = 8;
            title.style.marginBottom = 8;
            root.Add(title);
            var help = new HelpBox("상단 ▶ Play 후 각 축의 목표 각도를 입력하고 이동을 누르세요.", HelpBoxMessageType.Info);
            root.Add(help);
            root.Add(new Label("시험 범위 ±45° · 속도 20°/초 · 자체 수동 관절 제어"));
            var calculatedState = new Label { name = "fanuc-fk-calculated" };
            var actualState = new Label { name = "fanuc-fk-actual" };
            var errorState = new Label { name = "fanuc-fk-errors" };
            foreach (var label in new[] { calculatedState, actualState, errorState })
            {
                label.style.whiteSpace = WhiteSpace.Normal;
                label.style.marginTop = 8;
                root.Add(label);
            }
            var fkFailure = new HelpBox("", HelpBoxMessageType.Warning) { name = "fanuc-fk-failure" };
            fkFailure.style.display = DisplayStyle.None;
            root.Add(fkFailure);
            var ik = new Foldout { text = "IK 목표·관절 이동", value = true };
            ik.style.marginTop = 12;
            ik.Add(new HelpBox("입력은 미리보기이며 ‘목표로 이동’을 눌러야 적용됩니다. 요청 시 최신 상태로 다시 계산하고 속도 제한 안에서 6축이 함께 도착합니다. TCP 직선 경로·가속도·충돌 검사는 별도입니다.", HelpBoxMessageType.Info));
            var goalPosition = new Vector3Field("위치 (mm)") { name = "fanuc-ik-position", value = new Vector3(86.89688f, 986.3321f, -938.8343f) };
            ik.Add(goalPosition);
            ik.Add(new Label("방향 Quaternion (x, y, z, w)"));
            var goalRotation = new Vector4Field { name = "fanuc-ik-rotation", value = new Vector4(-.6363726f, -.73803055f, -.209741f, .07968455f),
                tooltip = "FK 표시와 같은 RobotFrame 기준 단위 Quaternion입니다. 현재 TCP 복사로 가져올 수 있습니다." };
            ik.Add(goalRotation);
            var seedPolicy = new DropdownField("초기값", new System.Collections.Generic.List<string> {
                "현재 자세 우선 + 고정 초기값 보완", "현재 자세만", "검증한 고정 초기값 3개" }, 0) { name = "fanuc-ik-policy" };
            ik.Add(seedPolicy);
            var copy = new Button(() =>
            {
                if (!lab.TryReadActualTcp(out var position, out var rotation, out _)) return;
                goalPosition.value = position * 1000f;
                goalRotation.value = new Vector4(rotation.x, rotation.y, rotation.z, rotation.w);
            }) { name = "fanuc-ik-copy", text = "현재 TCP를 목표로 복사" };
            ik.Add(copy);
            string commandError = null;
            var commands = new VisualElement();
            commands.style.flexDirection = FlexDirection.Row;
            var move = new Button(() =>
            {
                var r = goalRotation.value;
                commandError = lab.TryMoveTcp(goalPosition.value * .001f, new Quaternion(r.x, r.y, r.z, r.w),
                    (FanucJointLab.IkSeedPolicy)seedPolicy.index, out _, out var error) ? null : error;
            }) { name = "fanuc-ik-move", text = "목표로 이동 (관절)" };
            var stop = new Button(() => { lab.StopAll(); commandError = null; }) { name = "fanuc-ik-stop", text = "전체 정지" };
            move.style.flexGrow = stop.style.flexGrow = 1;
            move.style.minHeight = stop.style.minHeight = 28;
            commands.Add(move); commands.Add(stop); ik.Add(commands);
            var motionState = new Label { name = "fanuc-motion-state" };
            motionState.style.whiteSpace = WhiteSpace.Normal;
            ik.Add(motionState);
            var commandFailure = new HelpBox("", HelpBoxMessageType.Warning) { name = "fanuc-ik-command-failure" };
            commandFailure.style.display = DisplayStyle.None;
            ik.Add(commandFailure);
            var ikState = new Label { name = "fanuc-ik-result" };
            var ikJoints = new Label { name = "fanuc-ik-joints" };
            var ikSnapshot = new Label { name = "fanuc-ik-snapshot" };
            foreach (var label in new[] { ikState, ikJoints, ikSnapshot })
            {
                label.style.whiteSpace = WhiteSpace.Normal;
                label.style.marginTop = 6;
                ik.Add(label);
            }
            var ikFailure = new HelpBox("", HelpBoxMessageType.Warning) { name = "fanuc-ik-failure" };
            ikFailure.style.display = DisplayStyle.None;
            ik.Add(ikFailure);
            void InvalidateGoal()
            {
                move.SetEnabled(false);
                ikState.text = "입력 변경 · 계산 대기";
                ikJoints.text = "관절 해: 계산 대기";
                ikSnapshot.text = "새 입력으로 다시 계산합니다.";
                ikFailure.style.display = DisplayStyle.None;
            }
            goalPosition.RegisterValueChangedCallback(_ => InvalidateGoal());
            goalRotation.RegisterValueChangedCallback(_ => InvalidateGoal());
            seedPolicy.RegisterValueChangedCallback(_ => InvalidateGoal());
            root.Add(ik);
            var controls = new VisualElement();
            root.Add(controls);
            var states = new Label[6];
            var errors = new string[6];
            var manualButtons = new List<Button>();
            for (int i = 0; i < 6; i++)
            {
                int index = i;
                var row = new VisualElement();
                row.style.flexDirection = FlexDirection.Row;
                row.style.marginTop = 9;
                var name = new Label("A" + (i + 1));
                name.style.width = 28;
                name.style.alignSelf = Align.Center;
                row.Add(name);
                var angle = new FloatField { value = 15f, tooltip = "홈 자세에 상대적인 목표 각도(°)" };
                angle.style.width = 66;
                row.Add(angle);
                angle.RegisterValueChangedCallback(_ => errors[index] = null);
                var moveJoint = AddButton(row, "이동", () => errors[index] = lab.MoveJoint(index, angle.value) ? null : "요청 거부: 유한한 값과 허용 범위를 확인하세요.");
                moveJoint.name = "fanuc-joint-move-" + i; manualButtons.Add(moveJoint);
                var home = AddButton(row, "0° 복귀", () => { lab.MoveJoint(index, 0f); errors[index] = null; });
                home.name = "fanuc-joint-home-" + i; manualButtons.Add(home);
                AddButton(row, "정지", () => lab.StopJoint(index)).name = "fanuc-joint-stop-" + i;
                controls.Add(row);
                states[i] = new Label();
                states[i].style.whiteSpace = WhiteSpace.Normal;
                controls.Add(states[i]);
            }
            var all = new VisualElement();
            all.style.flexDirection = FlexDirection.Row;
            all.style.marginTop = 12;
            AddButton(all, "전체 정지", lab.StopAll).name = "fanuc-all-stop";
            var homeAll = AddButton(all, "전체 0° 복귀", lab.HomeAll); homeAll.name = "fanuc-all-home"; manualButtons.Add(homeAll);
            controls.Add(all);
            var geometrySettings = new Foldout { text = "FK 홈 기하 입력 (m·Quaternion·로컬 축)", value = false };
            geometrySettings.Add(new PropertyField(settingsObject.FindProperty("geometry"), "저장한 홈 기하"));
            root.Add(geometrySettings);
            var settings = new Foldout { text = "관절·기준점·TCP 연결", value = false };
            settings.style.marginTop = 10;
            settings.Add(new PropertyField(settingsObject.FindProperty("joints")));
            settings.Add(new PropertyField(settingsObject.FindProperty("robotFrame")));
            settings.Add(new PropertyField(settingsObject.FindProperty("tcp")));
            root.Add(settings);

            void Refresh()
            {
                if (lab == null) return;
                bool playing = Application.isPlaying;
                bool paused = EditorApplication.isPaused;
                bool manual = CellRunControllerEditor.ManualAllowed(lab, out var lockReason);
                controls.SetEnabled(playing);
                foreach (var button in manualButtons) button.SetEnabled(playing && !paused && Time.timeScale > 0f && lab.IsReady && manual);
                help.text = !playing ? "상단 ▶ Play 후 각 축의 목표 각도를 입력하고 이동을 누르세요."
                    : paused ? "일시정지 중에도 목표 미리보기는 가능합니다. 관절 조작은 상단 Ⅱ 버튼을 해제하세요."
                    : Time.timeScale <= 0f ? "시간 배율이 0입니다. 정지는 계속 가능합니다."
                    : !lab.IsReady ? "관절·TCP 연결 또는 축 활성 상태를 확인하세요."
                    : !manual ? lockReason
                    : "IK 목표 관절 이동 또는 기존 축별 조작을 사용하세요. 새 목표 입력은 이동 버튼을 누르기 전까지 미리보기입니다.";
                bool manualMoving = false;
                foreach (var joint in lab.Joints) if (joint != null) manualMoving |= joint.IsMoving;
                motionState.text = !playing ? "이동 상태: Play 대기"
                    : !string.IsNullOrEmpty(lab.LastMotionError) ? "이동 중단: " + lab.LastMotionError
                    : lab.IsTcpMoving ? paused || Time.timeScale <= 0f ? "관절 목표 이동: 일시정지 · 계획 유지" : "계획한 관절 목표로 이동 중"
                    : manualMoving ? "관절별 수동 이동 중" : "이동 상태: 대기·정지";
                commandFailure.text = string.IsNullOrEmpty(commandError) ? "" : "마지막 이동 요청 거부: " + commandError;
                commandFailure.style.display = string.IsNullOrEmpty(commandError) ? DisplayStyle.None : DisplayStyle.Flex;
                for (int i = 0; i < states.Length; i++)
                {
                    var joint = i < lab.Joints.Count ? lab.Joints[i] : null;
                    states[i].text = !playing ? "Play 대기"
                        : joint == null ? "축 연결 없음"
                        : $"현재 {joint.CurrentPosition:F1}° / 목표 {joint.TargetPosition:F1}° · {(joint.IsMoving ? "이동 중" : "정지")}"
                            + (string.IsNullOrEmpty(errors[i]) ? "" : "\n" + errors[i]);
                }
                if (!playing)
                {
                    calculatedState.text = "계산 TCP: Play 대기";
                    actualState.text = "실제 TCP: Play 대기";
                    errorState.text = "오차: Play 대기";
                    fkFailure.style.display = DisplayStyle.None;
                }
                else if (lab.TryReadFkDiagnostics(out var diagnostics, out var error))
                {
                    calculatedState.text = FormatTcp("계산 TCP", diagnostics.Calculated.PositionMetres, diagnostics.Calculated.Rotation);
                    actualState.text = FormatTcp("실제 TCP", diagnostics.ActualPositionMetres, diagnostics.ActualRotation);
                    errorState.text = $"위치 오차 {diagnostics.PositionErrorMetres * 1000f:F6} mm\n전체 회전 오차 {diagnostics.RotationErrorDegrees:F6}°";
                    fkFailure.style.display = DisplayStyle.None;
                }
                else
                {
                    calculatedState.text = "계산 TCP: 계산 불가";
                    actualState.text = lab.TryReadActualTcp(out var position, out var rotation, out var actualError)
                        ? FormatTcp("실제 TCP", position, rotation) : "실제 TCP: 읽기 불가 · " + actualError;
                    errorState.text = "오차: 계산 불가";
                    fkFailure.text = "FK 계산 불가.\n" + error;
                    fkFailure.style.display = DisplayStyle.Flex;
                }
            }
            void RefreshIk()
            {
                if (lab == null) return;
                bool playing = Application.isPlaying;
                bool ready = playing && lab.IsReady && lab.RobotFrame != null;
                stop.SetEnabled(playing);
                move.SetEnabled(false);
                goalPosition.SetEnabled(ready); goalRotation.SetEnabled(ready); seedPolicy.SetEnabled(ready); copy.SetEnabled(ready);
                if (!playing)
                {
                    ikState.text = "IK: Play 후 미리보기";
                    ikJoints.text = "관절 해: 대기";
                    ikSnapshot.text = "초기값: 대기";
                    ikFailure.style.display = DisplayStyle.None;
                    return;
                }
                var r = goalRotation.value;
                bool success = lab.TryPreviewTcp(goalPosition.value * .001f, new Quaternion(r.x, r.y, r.z, r.w),
                    (FanucJointLab.IkSeedPolicy)seedPolicy.index, out var preview, out var error);
                var result = preview.Result;
                move.SetEnabled(success && ready && !EditorApplication.isPaused && Time.timeScale > 0f && CellRunControllerEditor.ManualAllowed(lab, out _));
                ikState.text = success
                    ? $"계산 성공 · 위치 오차 {result.PositionErrorMetres.Value * 1000f:F4} mm · 전체 회전 오차 {result.RotationErrorDegrees.Value:F5}°"
                    : result != null && result.PositionErrorMetres.HasValue
                        ? $"계산 실패 · 실행 가능한 해 없음\n최선 잔차: 위치 {result.PositionErrorMetres.Value * 1000f:F4} mm · 전체 회전 {result.RotationErrorDegrees.Value:F5}°"
                        : "계산 실패 · 실행 가능한 해 없음";
                ikJoints.text = success ? "관절 해 (°): " + FormatJoints(result.JointDegrees) : "관절 해: 없음";
                ikSnapshot.text = result == null ? "초기값: 계산 전 연결·기하 확인 필요"
                    : $"{(success ? "선택" : "최선 잔차")} 초기값: {preview.SeedName} · 시도 {result.SeedsTried}/{preview.AvailableSeeds} · 반복 {result.Iterations} · {result.ElapsedMilliseconds:F3} ms\n"
                    + "계산 시 현재 관절 (°): " + FormatJoints(preview.CurrentJointDegrees);
                ikFailure.text = success ? "" : error;
                ikFailure.style.display = success ? DisplayStyle.None : DisplayStyle.Flex;
            }
            root.schedule.Execute(Refresh).Every(100);
            root.schedule.Execute(RefreshIk).Every(300);
            Refresh();
            RefreshIk();
            return root;
        }

        private static string FormatJoints(System.Collections.Generic.IReadOnlyList<float> values)
        {
            var text = new string[values.Count];
            for (int i = 0; i < text.Length; i++) text[i] = $"A{i + 1} {values[i]:F2}";
            return string.Join(" · ", text);
        }

        private static string FormatTcp(string label, Vector3 positionMetres, Quaternion rotation) =>
            $"{label} · RobotFrame\n위치 X {positionMetres.x * 1000f:F3} · Y {positionMetres.y * 1000f:F3} · Z {positionMetres.z * 1000f:F3} mm\n"
            + $"방향 q(x,y,z,w) = ({rotation.x:F6}, {rotation.y:F6}, {rotation.z:F6}, {rotation.w:F6})";

        private static Button AddButton(VisualElement parent, string text, System.Action action)
        {
            var button = new Button(action) { text = text };
            button.style.flexGrow = 1;
            button.style.minHeight = 25;
            parent.Add(button);
            return button;
        }
    }
}
