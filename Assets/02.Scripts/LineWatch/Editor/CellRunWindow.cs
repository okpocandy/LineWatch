using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    public sealed class CellRunWindow : EditorWindow
    {
        private void OnEnable()
        {
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
            EditorSceneManager.activeSceneChangedInEditMode += OnSceneChanged;
        }
        private void OnDisable()
        {
            EditorApplication.playModeStateChanged -= OnPlayModeChanged;
            EditorSceneManager.activeSceneChangedInEditMode -= OnSceneChanged;
        }
        public void CreateGUI() => RefreshCell();
        public void RefreshCell()
        {
            rootVisualElement.Clear();
            var cell = Object.FindFirstObjectByType<CellRunController>();
            if (cell == null)
            {
                rootVisualElement.Add(new HelpBox("LineWatch 씬의 셀 운전 관리자가 필요합니다. 작업 씬을 열고 다시 연결하세요.", HelpBoxMessageType.Info));
                rootVisualElement.Add(new Button(RefreshCell) { text = "씬에서 셀 다시 찾기" });
                return;
            }
            var scroll = new ScrollView { name = "cell-run-scroll" };
            scroll.style.paddingLeft = scroll.style.paddingRight = 10;
            scroll.style.paddingTop = scroll.style.paddingBottom = 10;
            scroll.Add(CellRunControllerEditor.CreateControls(cell));
            rootVisualElement.Add(scroll);
        }
        private void OnPlayModeChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.EnteredPlayMode || state == PlayModeStateChange.EnteredEditMode) RefreshCell();
        }
        private void OnSceneChanged(Scene previous, Scene current) => RefreshCell();
    }
}
