using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    [CustomEditor(typeof(FanucJointLab))]
    public sealed class FanucJointLabEditor : UnityEditor.Editor
    {
        [MenuItem("LineWatch/FANUC 조작 테스트")]
        private static void OpenLab()
        {
            var window = EditorWindow.GetWindow<FanucJointLabWindow>("FANUC 조작 테스트");
            window.minSize = new Vector2(400f, 560f);
            window.position = new Rect(100f, 100f, 460f, 650f);
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
            var title = new Label("FANUC 관절 조작 테스트");
            title.style.unityFontStyleAndWeight = FontStyle.Bold;
            title.style.fontSize = 16;
            title.style.marginTop = 8;
            title.style.marginBottom = 8;
            root.Add(title);
            var help = new HelpBox("상단 ▶ Play 후 각 축의 목표 각도를 입력하고 이동을 누르세요.", HelpBoxMessageType.Info);
            root.Add(help);
            root.Add(new Label("시험 범위 ±45° · 속도 20°/초 · 자체 수동 관절 제어"));
            var controls = new VisualElement();
            root.Add(controls);
            var states = new Label[6];
            var errors = new string[6];
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
                AddButton(row, "이동", () => errors[index] = lab.MoveJoint(index, angle.value) ? null : "요청 거부: 유한한 값과 허용 범위를 확인하세요.");
                AddButton(row, "0° 복귀", () => { lab.MoveJoint(index, 0f); errors[index] = null; });
                AddButton(row, "정지", () => lab.Joints[index].Stop());
                controls.Add(row);
                states[i] = new Label();
                states[i].style.whiteSpace = WhiteSpace.Normal;
                controls.Add(states[i]);
            }
            var all = new VisualElement();
            all.style.flexDirection = FlexDirection.Row;
            all.style.marginTop = 12;
            AddButton(all, "전체 정지", lab.StopAll);
            AddButton(all, "전체 0° 복귀", lab.HomeAll);
            controls.Add(all);
            var settings = new Foldout { text = "관절·TCP 연결", value = false };
            settings.style.marginTop = 10;
            settings.Add(new PropertyField(settingsObject.FindProperty("joints")));
            settings.Add(new PropertyField(settingsObject.FindProperty("tcp")));
            root.Add(settings);

            void Refresh()
            {
                if (lab == null) return;
                bool playing = Application.isPlaying;
                bool paused = EditorApplication.isPaused;
                controls.SetEnabled(playing && !paused && lab.IsReady);
                help.text = !playing ? "상단 ▶ Play 후 각 축의 목표 각도를 입력하고 이동을 누르세요."
                    : paused ? "일시정지 중입니다. 상단 Ⅱ 버튼을 해제하세요."
                    : !lab.IsReady ? "관절·TCP 연결 또는 축 활성 상태를 확인하세요."
                    : "시험 준비 완료. 공구 위치 계산·IK·충돌 회피는 아직 연결하지 않았습니다.";
                for (int i = 0; i < states.Length; i++)
                {
                    var joint = i < lab.Joints.Count ? lab.Joints[i] : null;
                    states[i].text = !playing ? "Play 대기"
                        : joint == null ? "축 연결 없음"
                        : $"현재 {joint.CurrentPosition:F1}° / 목표 {joint.TargetPosition:F1}° · {(joint.IsMoving ? "이동 중" : "정지")}"
                            + (string.IsNullOrEmpty(errors[i]) ? "" : "\n" + errors[i]);
                }
            }
            root.schedule.Execute(Refresh).Every(100);
            Refresh();
            return root;
        }

        private static void AddButton(VisualElement parent, string text, System.Action action)
        {
            var button = new Button(action) { text = text };
            button.style.flexGrow = 1;
            button.style.minHeight = 25;
            parent.Add(button);
        }
    }
}
