using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UIElements;

namespace LineWatch.Editor
{
    public sealed class CellRunWindow : EditorWindow
    {
        private LineWatchCellLayout equipmentLayout;
        private Foldout equipmentPanel;
        private int pickingControl;
        private Vector2 pickingStart;
        public CellEquipmentSelection EquipmentSelection { get; private set; }
        private void OnEnable()
        {
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
            EditorSceneManager.activeSceneChangedInEditMode += OnSceneChanged;
            SceneView.duringSceneGui += OnSceneGUI;
        }
        private void OnDisable()
        {
            EditorApplication.playModeStateChanged -= OnPlayModeChanged;
            EditorSceneManager.activeSceneChangedInEditMode -= OnSceneChanged;
            SceneView.duringSceneGui -= OnSceneGUI;
            if (pickingControl != 0 && GUIUtility.hotControl == pickingControl) GUIUtility.hotControl = 0;
            pickingControl = 0;
            SceneView.RepaintAll();
        }
        public void CreateGUI() => RefreshCell();
        public void RefreshCell()
        {
            rootVisualElement.Clear();
            equipmentPanel = null;
            var cell = Object.FindFirstObjectByType<CellRunController>();
            if (cell == null)
            {
                equipmentLayout = null; EquipmentSelection = null;
                rootVisualElement.Add(new HelpBox("LineWatch 씬의 셀 운전 관리자가 필요합니다. 작업 씬을 열고 다시 연결하세요.", HelpBoxMessageType.Info));
                rootVisualElement.Add(new Button(RefreshCell) { text = "씬에서 셀 다시 찾기" });
                return;
            }
            var scroll = new ScrollView { name = "cell-run-scroll" };
            scroll.style.paddingLeft = scroll.style.paddingRight = 10;
            scroll.style.paddingTop = scroll.style.paddingBottom = 10;
            scroll.Add(CellRunControllerEditor.CreateControls(cell));
            var layout = cell.GetComponent<LineWatchCellLayout>();
            if (equipmentLayout != layout || EquipmentSelection == null)
            { equipmentLayout = layout; EquipmentSelection = new CellEquipmentSelection(layout); }
            equipmentPanel = new Foldout { name = "cell-equipment-panel", text = "설비 목록 · 상세", value = false };
            equipmentPanel.Add(new HelpBox("펼친 동안 Scene 뷰에서 로봇/인계대를 왼쪽 클릭하면 목록과 함께 선택됩니다. 청록색 상자는 선택 범위이며 Alt 탐색은 유지됩니다.", HelpBoxMessageType.Info));
            equipmentPanel.Add(new CellEquipmentView(EquipmentSelection)); scroll.Add(equipmentPanel);
            var framingRequest = new HelpBox("", HelpBoxMessageType.Warning) { name = "equipment-frame-request" };
            framingRequest.style.display = DisplayStyle.None;
            var frame = new Button(() =>
            {
                bool framed = TryFrameSelectedEquipment(out var error);
                framingRequest.text = error ?? "";
                framingRequest.style.display = framed ? DisplayStyle.None : DisplayStyle.Flex;
            }) { name = "equipment-frame-selected", text = "선택 설비 보기 (Scene)" };
            frame.style.minHeight = 28;
            frame.tooltip = "마지막으로 사용한 Scene 뷰에서 선택 설비를 봅니다. Game 카메라는 이동하지 않습니다.";
            equipmentPanel.Add(frame); equipmentPanel.Add(framingRequest);
            void RefreshFrameAvailability() => frame.SetEnabled(SceneView.lastActiveSceneView != null && EquipmentSelection.TryGetSelectedBounds(out _));
            RefreshFrameAvailability();
            equipmentPanel.RegisterValueChangedCallback(_ => SceneView.RepaintAll());
            equipmentPanel.schedule.Execute(() =>
            {
                RefreshFrameAvailability();
                if (equipmentPanel != null && equipmentPanel.value) SceneView.RepaintAll();
            }).Every(100);
            rootVisualElement.Add(scroll);
        }
        public bool TryFrameSelectedEquipment(out string error)
        {
            if (EquipmentSelection == null || !EquipmentSelection.TryGetSelectedBounds(out var bounds))
            { error = "화면에서 볼 수 있는 설비를 먼저 선택하세요."; return false; }
            var sceneView = SceneView.lastActiveSceneView;
            if (sceneView == null)
            { error = "Scene 뷰를 열고 다시 시도하세요."; return false; }
            if (!sceneView.Frame(bounds, false))
            { error = "선택 설비를 Scene 뷰에 표시할 수 없습니다."; return false; }
            error = null;
            return true;
        }
        private void OnSceneGUI(SceneView sceneView)
        {
            var evt = Event.current;
            bool enabled = equipmentPanel != null && equipmentPanel.value && EquipmentSelection != null;
            if (pickingControl != 0 && GUIUtility.hotControl == pickingControl && evt.type == EventType.MouseUp && evt.button == 0)
            {
                if (enabled && !evt.alt && !Tools.viewToolActive && Vector2.Distance(pickingStart, evt.mousePosition) < 5f)
                {
                    var hit = HandleUtility.PickGameObject(evt.mousePosition, false);
                    if (EquipmentSelection.TrySelectTarget(hit != null ? hit.transform : null, out _))
                        rootVisualElement.Q<CellEquipmentView>()?.Refresh();
                }
                GUIUtility.hotControl = 0; pickingControl = 0; evt.Use(); sceneView.Repaint();
                return;
            }
            if (!enabled) return;
            if (evt.type == EventType.Repaint)
            {
                if (EquipmentSelection.TryGetSelectedBounds(out var bounds))
                    using (new Handles.DrawingScope(Color.cyan)) Handles.DrawWireCube(bounds.center, bounds.size);
                return;
            }
            int control = GUIUtility.GetControlID(FocusType.Passive);
            if (!evt.alt && !Tools.viewToolActive && !evt.shift && !evt.control && !evt.command)
            {
                if (evt.type == EventType.Layout) HandleUtility.AddDefaultControl(control);
                if (evt.type == EventType.MouseDown && evt.button == 0 && GUIUtility.hotControl == 0 && HandleUtility.nearestControl == control)
                {
                    pickingStart = evt.mousePosition; pickingControl = control; GUIUtility.hotControl = control; evt.Use();
                }
            }
        }
        private void OnPlayModeChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.EnteredPlayMode || state == PlayModeStateChange.EnteredEditMode) RefreshCell();
        }
        private void OnSceneChanged(Scene previous, Scene current) => RefreshCell();
    }
}
