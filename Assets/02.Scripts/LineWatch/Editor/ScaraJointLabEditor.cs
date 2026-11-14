using System.Collections.Generic;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    [CustomEditor(typeof(ScaraJointLab))]
    public sealed class ScaraJointLabEditor : UnityEditor.Editor
    {
        [MenuItem("LineWatch/SCARA 4축 조작 테스트")]
        private static void OpenLab()
        {
            var window = EditorWindow.GetWindow<ScaraJointLabWindow>("SCARA 4축 조작 테스트");
            window.minSize = new Vector2(440f, 540f);
            window.position = new Rect(120f, 60f, 530f, 880f);
            window.Show();
            window.RefreshLab();
            window.Focus();
        }

        public override VisualElement CreateInspectorGUI()
        {
            return CreateControls((ScaraJointLab)target, serializedObject);
        }

        internal static VisualElement CreateControls(ScaraJointLab lab, SerializedObject settingsObject)
        {
            var root = new VisualElement();
            var heading = new Label("SCARA 4축 조작 테스트");
            heading.style.unityFontStyleAndWeight = FontStyle.Bold;
            heading.style.fontSize = 16;
            heading.style.marginTop = 8;
            heading.style.marginBottom = 8;
            root.Add(heading);
            var help = new HelpBox("상단 ▶ Play 후 목표값을 입력하고 이동을 누르세요.", HelpBoxMessageType.Info);
            root.Add(help);
            root.Add(CreateIkControls(lab));
            var jointHeading = new Label("관절별 수동 조작");
            jointHeading.style.unityFontStyleAndWeight = FontStyle.Bold;
            jointHeading.style.marginTop = 12;
            root.Add(jointHeading);
            var units = new Label("1·2·4축: ° / 3축: m (0.050m = 아래로 50mm)");
            units.style.whiteSpace = WhiteSpace.Normal;
            root.Add(units);
            var controls = new VisualElement();
            root.Add(controls);
            var rotary = new[] { lab.Axis1, lab.Axis2, null, lab.Axis4 };
            var states = new Label[4];
            var errors = new string[4];
            for (int i = 0; i < 4; i++)
            {
                int index = i;
                var row = new VisualElement();
                row.style.flexDirection = FlexDirection.Row;
                row.style.marginTop = 12;
                var label = new Label($"{i + 1}축 {(i == 2 ? "m" : "°")}");
                label.style.width = 50;
                label.style.alignSelf = Align.Center;
                row.Add(label);
                var targetValue = new FloatField { value = i == 2 ? 0.05f : 20f,
                    name = "scara-joint-target-" + i,
                    tooltip = i == 2 ? "홈에서 아래로 이동할 거리(m), 0~0.150" : "홈 기준 상대 회전각(°)" };
                targetValue.style.width = 76;
                row.Add(targetValue);
                targetValue.RegisterValueChangedCallback(_ => errors[index] = null);
                AddButton(row, "이동", () => errors[index] = lab.MoveJoint(index, targetValue.value)
                    ? null : "요청 거부: 단위·허용 범위·유효한 숫자인지 확인하세요.");
                AddButton(row, "홈 복귀", () => { lab.MoveJoint(index, 0f); errors[index] = null; });
                AddButton(row, "정지", () => lab.StopJoint(index));
                controls.Add(row);
                states[i] = new Label();
                states[i].style.whiteSpace = WhiteSpace.Normal;
                controls.Add(states[i]);
            }
            var all = new VisualElement();
            all.style.flexDirection = FlexDirection.Row;
            all.style.marginTop = 12;
            AddButton(all, "전체 정지", lab.StopAll);
            AddButton(all, "전체 홈 복귀", lab.HomeAll);
            controls.Add(all);
            var calculatedState = new Label { name = "scara-fk-calculated" };
            var actualState = new Label { name = "scara-fk-actual" };
            var errorState = new Label { name = "scara-fk-errors" };
            foreach (var label in new[] { calculatedState, actualState, errorState })
            {
                label.style.whiteSpace = WhiteSpace.Normal;
                label.style.marginTop = 8;
                root.Add(label);
            }
            var fkFailure = new HelpBox("", HelpBoxMessageType.Warning) { name = "scara-fk-failure" };
            fkFailure.style.display = DisplayStyle.None;
            root.Add(fkFailure);
            var geometrySettings = new Foldout { text = "FK 기하 입력 (홈·로봇 기준)", value = false };
            var geometryFields = new[] { "fkLink1Metres", "fkLink2Metres", "fkHeight12Metres", "fkHeight23Metres", "fkToolOffsetMetres", "fkHomeToolYawDegrees" };
            var geometryLabels = new[] { "L1 (m)", "L2 (m)", "링크 1 높이 차 (m)", "링크 2 높이 차 (m)", "홈 도구 벡터 (m)", "홈 도구 수평각 (°)" };
            for (int i = 0; i < geometryFields.Length; i++)
                geometrySettings.Add(new PropertyField(settingsObject.FindProperty(geometryFields[i]), geometryLabels[i]));
            root.Add(geometrySettings);
            var settings = new Foldout { text = "축·기준점 연결", value = false };
            foreach (string field in new[] { "axis1", "axis2", "axis3", "axis4", "robotFrame", "tcp" })
                settings.Add(new PropertyField(settingsObject.FindProperty(field)));
            root.Add(settings);

            void Refresh()
            {
                if (lab == null) return;
                bool playing = Application.isPlaying;
                bool paused = EditorApplication.isPaused;
                controls.SetEnabled(playing && !paused && lab.IsReady);
                help.text = !playing ? "상단 ▶ Play 후 목표값을 입력하고 이동을 누르세요."
                    : paused ? "일시정지 중입니다. 상단 Ⅱ 버튼을 해제하세요."
                    : !lab.IsReady ? "네 축과 RobotFrame·TCP 연결을 확인하세요."
                    : "IK 목표 이동 또는 관절별 수동 조작을 사용하세요. 아래 FK 진단은 현재 관절값을 읽습니다.";
                for (int i = 0; i < states.Length; i++)
                {
                    if (!lab.IsReady) { states[i].text = "축 연결 확인 필요"; continue; }
                    bool slide = i == 2;
                    float current = slide ? lab.Axis3.CurrentPosition : rotary[i].CurrentPosition;
                    float target = slide ? lab.Axis3.TargetPosition : rotary[i].TargetPosition;
                    bool moving = slide ? lab.Axis3.IsMoving : rotary[i].IsMoving;
                    string range = slide ? "0~0.150m · 50mm/초" : $"{rotary[i].MinimumDegrees:0}~{rotary[i].MaximumDegrees:0}° · {rotary[i].SpeedDegreesPerSecond:0}°/초";
                    states[i].text = !playing ? "Play 대기 · 범위 " + range
                        : $"현재 {current:F3} / 목표 {target:F3}{(slide ? "m" : "°")} · {(moving ? "이동 중" : "정지")}"
                            + "\n범위 " + range + (string.IsNullOrEmpty(errors[i]) ? "" : "\n" + errors[i]);
                }
                if (playing && lab.IsReady)
                {
                    if (lab.TryReadFkDiagnostics(out var diagnostics, out var error))
                    {
                        calculatedState.text = FormatTcp("계산 TCP", diagnostics.Calculated.PositionMetres, diagnostics.Calculated.YawDegrees);
                        actualState.text = FormatTcp("실제 TCP", diagnostics.ActualPositionMetres, diagnostics.ActualYawDegrees);
                        errorState.text = $"위치 오차 {diagnostics.PositionErrorMetres * 1000f:F6} mm\n수평각 오차 {diagnostics.YawErrorDegrees:F6}°";
                        fkFailure.style.display = DisplayStyle.None;
                    }
                    else
                    {
                        calculatedState.text = "계산 TCP: 계산 불가";
                        actualState.text = FormatTcp("실제 TCP", lab.ActualTcpPosition, lab.ActualTcpYawDegrees);
                        errorState.text = "오차: 계산 불가";
                        fkFailure.text = "FK 계산 불가. FK 기하 입력을 확인하세요.\n" + error;
                        fkFailure.style.display = DisplayStyle.Flex;
                    }
                }
                else
                {
                    calculatedState.text = "계산 TCP: Play 대기";
                    actualState.text = "실제 TCP: Play 대기";
                    errorState.text = "오차: Play 대기";
                    fkFailure.style.display = DisplayStyle.None;
                }
            }
            root.schedule.Execute(Refresh).Every(100);
            Refresh();
            return root;
        }

        private static VisualElement CreateIkControls(ScaraJointLab lab)
        {
            var root = new VisualElement { name = "scara-ik-controls" };
            var heading = new Label("IK 목표 이동 · 로봇 기준");
            heading.style.unityFontStyleAndWeight = FontStyle.Bold;
            heading.style.marginTop = 12;
            root.Add(heading);
            root.Add(new HelpBox("목표 XYZ는 mm, 수평각은 °입니다. 각 축이 자기 속도로 움직여 TCP 직선 경로와 충돌 회피는 보장하지 않습니다.", HelpBoxMessageType.Info));
            var inputs = new VisualElement();
            var x = new FloatField("X (mm)") { value = 400f, name = "scara-ik-x" };
            var y = new FloatField("Y (mm)") { value = -200f, name = "scara-ik-y" };
            var z = new FloatField("Z (mm)") { value = -300f, name = "scara-ik-z" };
            var yaw = new FloatField("수평각 (°)") { value = 0f, name = "scara-ik-yaw" };
            foreach (var field in new[] { x, y, z, yaw }) inputs.Add(field);
            var selection = new DropdownField("해 선택", new List<string> {
                "현재 자세에서 가까운 해", "q2 양수 해", "q2 음수 해" }, 0) { name = "scara-ik-selection" };
            inputs.Add(selection);
            root.Add(inputs);
            var candidates = new Label { name = "scara-ik-candidates" };
            var chosen = new Label { name = "scara-ik-selected" };
            var status = new Label { name = "scara-ik-status" };
            var feedback = new HelpBox("", HelpBoxMessageType.Warning) { name = "scara-ik-feedback" };
            foreach (var label in new[] { candidates, chosen, status })
            {
                label.style.whiteSpace = WhiteSpace.Normal;
                label.style.marginTop = 6;
            }
            var row = new VisualElement();
            row.style.flexDirection = FlexDirection.Row;
            row.style.marginTop = 6;
            var move = new Button { text = "목표로 이동", name = "scara-ik-move" };
            var stop = new Button(() => { lab.StopAll(); status.text = "전체 정지 요청: 현재 자세를 유지합니다."; })
                { text = "전체 정지", name = "scara-ik-stop" };
            move.clicked += () => {
                bool accepted = lab.TryMoveTcp(Target(), yaw.value, Mode(), out var solution, out var error);
                status.text = accepted ? "마지막 요청 수락: " + FormatIkSolution(solution) : "요청 거부: " + error;
                Refresh();
            };
            foreach (var button in new[] { move, stop })
            {
                button.style.flexGrow = 1;
                button.style.minHeight = 28;
                row.Add(button);
            }
            root.Add(row);
            root.Add(candidates);
            root.Add(chosen);
            root.Add(feedback);
            root.Add(status);

            Vector3 Target() => new Vector3(x.value, y.value, z.value) * 0.001f;
            ScaraJointLab.IkSelection Mode() => (ScaraJointLab.IkSelection)selection.index;
            void Refresh()
            {
                if (lab == null) { root.SetEnabled(false); return; }
                bool ready = Application.isPlaying && lab.IsReady;
                bool canMove = ready && !EditorApplication.isPaused && Time.timeScale > 0f;
                inputs.SetEnabled(canMove);
                stop.SetEnabled(canMove);
                move.SetEnabled(false);
                feedback.style.display = DisplayStyle.None;
                if (!ready)
                {
                    candidates.text = "후보: Play 상태와 네 축 연결을 확인하세요.";
                    chosen.text = "선택 예정: 없음";
                    status.text = "";
                    return;
                }
                bool valid = lab.TrySelectTcpSolution(Target(), yaw.value, Mode(), out var result, out var solution, out var error);
                candidates.text = result != null && result.Success ? $"후보 {result.Solutions.Count}개" : "후보 없음";
                if (result != null && result.Success)
                    foreach (var candidate in result.Solutions) candidates.text += "\n" + FormatIkSolution(candidate);
                chosen.text = valid ? "선택 예정: " + FormatIkSolution(solution) : "선택 예정: 없음";
                move.SetEnabled(canMove && valid);
                if (!valid || solution.IsSingular || !canMove)
                {
                    feedback.text = !canMove ? "일시정지를 해제한 후 이동하세요."
                        : !valid ? error : "특이 자세: 굽힘 분기가 합쳐지는 자세입니다. 속도 제어·회피 경로는 지원하지 않습니다.";
                    feedback.style.display = DisplayStyle.Flex;
                }
            }
            foreach (var field in new[] { x, y, z, yaw })
                field.RegisterValueChangedCallback(_ => { status.text = ""; Refresh(); });
            selection.RegisterValueChangedCallback(_ => { status.text = ""; Refresh(); });
            root.schedule.Execute(Refresh).Every(100);
            Refresh();
            return root;
        }

        private static string FormatIkSolution(ScaraInverseKinematics.Solution solution)
        {
            var j = solution.Joints;
            string branch = solution.ElbowSign > 0 ? "q2 양수" : solution.ElbowSign < 0 ? "q2 음수" : "특이 자세";
            return $"{branch} · ({j.Q1Degrees:F2}°, {j.Q2Degrees:F2}°, {j.D3Metres * 1000f:F3}mm, {j.Q4Degrees:F2}°)";
        }

        private static string FormatTcp(string title, Vector3 positionMetres, float yawDegrees)
        {
            var mm = positionMetres * 1000f;
            return $"{title} (로봇 기준)\nX {mm.x:F2} / Y {mm.y:F2} / Z {mm.z:F2} mm · 수평 {yawDegrees:F2}°";
        }

        private static void AddButton(VisualElement parent, string text, System.Action action)
        {
            var button = new Button(action) { text = text };
            button.style.flexGrow = 1;
            button.style.minHeight = 26;
            parent.Add(button);
        }
    }
}
