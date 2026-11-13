using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    [CustomEditor(typeof(MachineControlLab))]
    public sealed class MachineControlLabEditor : UnityEditor.Editor
    {
        [MenuItem("LineWatch/조작 테스트")]
        private static void SelectLab()
        {
            var lab = Object.FindFirstObjectByType<MachineControlLab>();
            if (lab == null)
            {
                EditorUtility.DisplayDialog("LineWatch 조작 테스트", "RobotLab 씬을 열어주세요.", "확인");
                return;
            }

            Selection.activeGameObject = lab.gameObject;
            EditorGUIUtility.PingObject(lab.gameObject);
            EditorUtility.OpenPropertyEditor(lab);
        }

        public override VisualElement CreateInspectorGUI()
        {
            var lab = (MachineControlLab)target;
            var root = new VisualElement();
            var heading = new Label("로봇·센서 조작 테스트");
            heading.style.unityFontStyleAndWeight = FontStyle.Bold;
            heading.style.fontSize = 16;
            heading.style.marginTop = 8;
            heading.style.marginBottom = 8;
            root.Add(heading);

            var instruction = new HelpBox("상단 ▶ Play를 누른 뒤 아래 버튼으로 조작하세요.", HelpBoxMessageType.Info);
            root.Add(instruction);
            var controls = new VisualElement { name = "lab-controls" };
            root.Add(controls);
            var moveButton = AddButton(controls, "축 이동", "move-axis", lab.MoveToTestAngle);
            AddButton(controls, "축 0° 복귀", "home-axis", lab.MoveHome);
            AddButton(controls, "축 정지", "stop-axis", lab.StopAxis);
            AddButton(controls, "시험 물체 → 센서 안", "probe-inside", lab.MoveProbeInside);
            AddButton(controls, "시험 물체 → 센서 밖", "probe-outside", lab.MoveProbeOutside);

            var axisState = new Label { name = "axis-state" };
            var sensorState = new Label { name = "sensor-state" };
            axisState.style.marginTop = 8;
            root.Add(axisState);
            root.Add(sensorState);

            var settings = new Foldout { text = "연결·시험 설정", value = false };
            settings.style.marginTop = 10;
            foreach (string property in new[] { "axis", "sensor", "probe", "testAngleDegrees" })
                settings.Add(new PropertyField(serializedObject.FindProperty(property)));
            root.Add(settings);

            void Refresh()
            {
                if (lab == null) return;
                bool playing = Application.isPlaying;
                bool paused = EditorApplication.isPaused;
                moveButton.text = $"축 {lab.TestAngleDegrees:0.#}° 이동";
                controls.SetEnabled(playing && !paused && lab.IsReady);
                instruction.text = !playing ? "상단 ▶ Play를 누른 뒤 아래 버튼으로 조작하세요."
                    : paused ? "일시정지 중입니다. 상단 Ⅱ 버튼을 해제하면 조작할 수 있습니다."
                    : !lab.IsReady ? "시험 연결이 누락됐습니다. 아래 연결·시험 설정을 확인하세요."
                    : "시험 준비 완료. 이동·복귀·정지 또는 센서 버튼을 누르세요.";
                axisState.text = playing && lab.Axis != null
                    ? $"축: {lab.Axis.CurrentPosition:F1}° / 목표 {lab.Axis.TargetPosition:F1}° · {(lab.Axis.IsMoving ? "이동 중" : "정지")}"
                    : "축: Play 대기";
                sensorState.text = playing && lab.TestSensor != null
                    ? $"센서: {(lab.TestSensor.Occupied ? "감지됨" : "비어 있음")} · 진입 {lab.TestSensor.EnterCount} / 이탈 {lab.TestSensor.ExitCount}"
                    : "센서: Play 대기";
            }

            root.schedule.Execute(Refresh).Every(100);
            Refresh();
            return root;
        }

        private static Button AddButton(VisualElement parent, string text, string name, System.Action action)
        {
            var button = new Button(action) { text = text, name = name };
            button.style.height = 32;
            button.style.marginTop = 3;
            parent.Add(button);
            return button;
        }
    }
}
