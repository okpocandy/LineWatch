using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    public sealed class FanucJointLabWindow : EditorWindow
    {
        private void OnEnable()
        {
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
        }

        private void OnDisable()
        {
            EditorApplication.playModeStateChanged -= OnPlayModeChanged;
        }

        public void CreateGUI()
        {
            RefreshLab();
        }

        public void RefreshLab()
        {
            rootVisualElement.Unbind();
            rootVisualElement.Clear();
            var lab = Object.FindFirstObjectByType<FanucJointLab>();
            if (lab == null)
            {
                rootVisualElement.Add(new HelpBox("RobotLab 씬을 열고 아래 버튼으로 다시 연결하세요.", HelpBoxMessageType.Info));
                rootVisualElement.Add(new Button(RefreshLab) { text = "씬에서 FANUC 다시 찾기" });
                return;
            }
            var settings = new SerializedObject(lab);
            var scroll = new ScrollView();
            scroll.style.paddingLeft = 10;
            scroll.style.paddingRight = 10;
            scroll.Add(FanucJointLabEditor.CreateControls(lab, settings));
            rootVisualElement.Add(scroll);
            rootVisualElement.Bind(settings);
        }

        private void OnPlayModeChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.EnteredPlayMode || state == PlayModeStateChange.EnteredEditMode)
                RefreshLab();
        }
    }
}
