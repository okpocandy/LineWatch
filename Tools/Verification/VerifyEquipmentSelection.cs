using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

public static class VerifyEquipmentSelection
{
    public static Task<object> Run() => Check(false);
    public static Task<object> Ui() => Check(true);

    private static async Task<object> Check(bool verifyUi)
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var c = layout.GetComponent<CellRunController>(); var t = layout.GetComponent<CellProductTracker>();
        var s = layout.Scara; var f = layout.Fanuc; var camera = Camera.main;
        var watch = Stopwatch.StartNew(); while (t.Count != 1) { Require(watch.Elapsed.TotalSeconds < 10, "Initial sample timeout."); await Task.Delay(1); }
        await Task.Delay(60); string before = Capture();
        var model = new CellEquipmentSelection(layout); CellRunWindow window = null; CellEquipmentView view = null; int uiReads = 0;
        if (verifyUi)
        {
            window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
            model = window.EquipmentSelection; view = window.rootVisualElement.Q<CellEquipmentView>("cell-equipment-view");
            var panel = window.rootVisualElement.Q<Foldout>("cell-equipment-panel");
            Require(panel != null && !panel.value && view != null, "Equipment panel is missing or clutters the default window.");
            ReadUi(null);
        }
        var entries = model.Snapshot();
        Require(entries.Length == 3 && entries.Select(e => e.Id).Distinct().Count() == 3 && entries.All(e => e.IsAvailable), "Equipment catalog mismatch.");
        Require(entries[0].Id == layout.ScaraId && entries[0].Kind == CellEquipmentSelection.EquipmentKind.Scara && entries[0].AxisCount == 4 && entries[0].Target == s.transform &&
            entries[1].Id == layout.FanucId && entries[1].Kind == CellEquipmentSelection.EquipmentKind.Fanuc && entries[1].AxisCount == 6 && entries[1].Target == f.transform &&
            entries[2].Id == CellEquipmentSelection.HandoffId && entries[2].Kind == CellEquipmentSelection.EquipmentKind.Handoff && entries[2].AxisCount == 0 && entries[2].Target == layout.HandoffPoint,
            "ID/type/axis-count/scene target mapping mismatch.");
        foreach (var expected in entries)
        {
            Require(model.TrySelect(expected.Id, out var e), e);
            Require(model.TryGetSelected(out var selected) && selected.Id == expected.Id && selected.Kind == expected.Kind && selected.AxisCount == expected.AxisCount && selected.Target == expected.Target,
                "Selected detail metadata differs from the catalog.");
            if (verifyUi) ReadUi(expected);
        }
        string previous = model.SelectedId;
        Require(!model.TrySelect("UNKNOWN-EQUIPMENT", out var error) && !string.IsNullOrEmpty(error) && model.SelectedId == previous && model.TryGetSelected(out _),
            "Invalid selection changed the existing selection.");
        if (verifyUi)
        {
            ReadUi(entries[2]); window.RefreshCell();
            Require(ReferenceEquals(window.EquipmentSelection, model) && model.SelectedId == previous, "Window refresh lost the selection.");
            view = window.rootVisualElement.Q<CellEquipmentView>("cell-equipment-view"); ReadUi(entries[2]);
        }
        model.ClearSelection(); Require(model.SelectedId == null && !model.TryGetSelected(out _), "Selection did not clear.");
        if (verifyUi) ReadUi(null);
        Require(Capture() == before, "Selection changed cell/products/robot commands or the camera.");
        var report = new { passed = true, normalSelectionFlows = 1, validSelections = 3, invalidIdRejected = true, selectionCleared = true,
            runtimeAndCameraPreserved = true, uiReads, windowRebuildChecked = verifyUi, entries = entries.Select(e => new { e.Id, kind = e.Kind.ToString(), e.AxisCount, target = e.Target.name }),
            directClicks = 0, callbacksOrUiInputInjected = 0 };
        Directory.CreateDirectory("Logs/LW13"); File.WriteAllText(verifyUi ? "Logs/LW13/equipment-ui.json" : "Logs/LW13/equipment-selection.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;

        void ReadUi(CellEquipmentSelection.EquipmentInfo? expected)
        {
            view.Refresh();
            Require(view.Q<Button>("equipment-clear").enabledInHierarchy == expected.HasValue, "Clear selection eligibility mismatch.");
            if (!expected.HasValue)
                Require(view.Q<Label>("equipment-detail-id").text == "ID: -" && view.Q<Label>("equipment-selection-summary").text.Contains("없음"), "Cleared detail did not reset.");
            else
            {
                var info = expected.Value;
                string type = info.Kind == CellEquipmentSelection.EquipmentKind.Scara ? "SCARA" : info.Kind == CellEquipmentSelection.EquipmentKind.Fanuc ? "FANUC" : "인계대";
                Require(view.Q<Label>("equipment-detail-id").text == "ID: " + info.Id &&
                    view.Q<Label>("equipment-detail-kind").text == "기종: " + type && view.Q<Label>("equipment-detail-axes").text == "축 수: " + info.AxisCount &&
                    view.Q<Label>("equipment-detail-cell-state").text == "셀 운전: " + c.State, "Selected visible metadata/cell state mismatch.");
                Require(!string.IsNullOrEmpty(view.Q<Label>("equipment-detail-state").text) && !string.IsNullOrEmpty(view.Q<Label>("equipment-detail-product").text), "Equipment status detail missing.");
            }
            uiReads++;
        }

        string Capture() => JsonConvert.SerializeObject(new
        {
            c.State, c.FaultReason, c.HasControlledSequence, c.AutomaticPhase, c.AutomaticProductId, c.IsAutomaticContinuous,
            records = t.Snapshot(), fifo = t.WaitingSnapshot(), slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            scara = new { s.CommandVersion, s.IsTrajectoryMoving, p = new[] { s.Axis1.CurrentPosition, s.Axis2.CurrentPosition, s.Axis3.CurrentPosition, s.Axis4.CurrentPosition },
                target = new[] { s.Axis1.TargetPosition, s.Axis2.TargetPosition, s.Axis3.TargetPosition, s.Axis4.TargetPosition } },
            fanuc = new { f.CommandVersion, f.IsTcpMoving, p = f.Joints.Select(j => j.CurrentPosition), target = f.Joints.Select(j => j.TargetPosition) },
            products = UnityEngine.Object.FindObjectsByType<GrippableItem>(FindObjectsSortMode.InstanceID).Select(i => new { i.ProductId, owner = i.Owner != null ? i.Owner.GetInstanceID() : 0,
                p = new[] { i.Body.position.x, i.Body.position.y, i.Body.position.z }, r = new[] { i.Body.rotation.x, i.Body.rotation.y, i.Body.rotation.z, i.Body.rotation.w } }),
            camera = camera != null ? (object)new { p = new[] { camera.transform.position.x, camera.transform.position.y, camera.transform.position.z },
                r = new[] { camera.transform.rotation.x, camera.transform.rotation.y, camera.transform.rotation.z, camera.transform.rotation.w } } : null
        });
    }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
