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
            window.position = new Rect(120f, 100f, 500f, 620f);
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
            var actualState = new Label();
            actualState.style.whiteSpace = WhiteSpace.Normal;
            actualState.style.marginTop = 8;
            root.Add(actualState);
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
                    : "실제 TCP는 로봇 기준입니다. FK·IK·충돌 회피는 미구현입니다.";
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
                actualState.text = playing && lab.IsReady
                    ? FormatTcp("실제 TCP", lab.ActualTcpPosition, lab.ActualTcpYawDegrees)
                    : "실제 TCP: Play 대기";
            }
            root.schedule.Execute(Refresh).Every(100);
            Refresh();
            return root;
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
