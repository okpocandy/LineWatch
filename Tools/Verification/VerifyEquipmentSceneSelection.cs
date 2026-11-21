using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UIElements;

public static class VerifyEquipmentSceneSelection
{
    public static object ConfigureScene()
    {
        Require(!EditorApplication.isPlaying, "Configure in Edit mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity" && !layout.gameObject.scene.isDirty, "Use clean LineWatch scene.");
        var seats = layout.GetComponentsInChildren<Transform>(true).Where(t => t.name == "Handoff_Point_Seat").ToArray();
        Require(seats.Length == 1 && seats[0].GetComponentInChildren<Renderer>() != null, "Need unique rendered handoff seat.");
        var data = new SerializedObject(layout);
        data.FindProperty("handoffEquipment").objectReferenceValue = seats[0];
        data.ApplyModifiedProperties();
        EditorSceneManager.MarkSceneDirty(layout.gameObject.scene);
        Require(EditorSceneManager.SaveScene(layout.gameObject.scene), "Scene save failed.");
        return new { configured = true, target = layout.HandoffEquipment.name };
    }

    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var c = layout.GetComponent<CellRunController>(); var t = layout.GetComponent<CellProductTracker>();
        var s = layout.Scara; var f = layout.Fanuc; var camera = Camera.main;
        var watch = Stopwatch.StartNew();
        while (t.Count != 1) { Require(watch.Elapsed.TotalSeconds < 10, "Initial sample timeout."); await Task.Delay(1); }
        await Task.Delay(60);
        string before = Capture();
        var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.RefreshCell();
        var model = window.EquipmentSelection;
        var view = window.rootVisualElement.Q<CellEquipmentView>();
        Require(model != null && view != null && layout.HandoffEquipment != null, "Selection references missing.");
        int targetMappings = 0;
        CheckTarget(s.GetComponentInChildren<Renderer>().transform, layout.ScaraId);
        CheckTarget(f.GetComponentInChildren<Renderer>().transform, layout.FanucId);
        CheckTarget(layout.HandoffEquipment.GetComponentInChildren<Renderer>().transform, CellEquipmentSelection.HandoffId);
        CheckTarget(layout.HandoffZone.GetComponentInChildren<Renderer>().transform, CellEquipmentSelection.HandoffId);
        Require(model.TrySelect(layout.ScaraId, out var error) && model.TryGetSelectedBounds(out _), error ?? "List selection has no highlight bounds.");
        var product = UnityEngine.Object.FindFirstObjectByType<GrippableItem>();
        Require(product != null && !model.TrySelectTarget(product.transform, out _) && !model.TrySelectTarget(layout.InputStop, out _) && model.SelectedId == layout.ScaraId,
            "Unsupported target changed equipment selection.");
        model.ClearSelection(); view.Refresh();
        Require(!model.TryGetSelectedBounds(out _) && view.Q<Label>("equipment-detail-id").text == "ID: -", "Clear left highlight/detail behind.");
        Require(Capture() == before, "Selection changed operation, product, robot or camera state.");
        var report = new { passed = true, targetMappings, listSelectionHasBounds = true, unsupportedTargetsRejected = 2, selectionCleared = true,
            runtimeAndCameraPreserved = true, directClicks = 0, callbacksOrUiInputInjected = 0, actualSceneClickAndOutlineAppearance = "User verification pending" };
        Directory.CreateDirectory("Logs/LW13"); File.WriteAllText("Logs/LW13/equipment-scene-selection.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;

        void CheckTarget(Transform target, string id)
        {
            Require(model.TrySelectTarget(target, out var e) && model.SelectedId == id, e ?? "Target mapped to wrong ID.");
            view.Refresh();
            Require(view.Q<Label>("equipment-detail-id").text == "ID: " + id, "Target selection did not update details.");
            var highlighted = id == CellEquipmentSelection.HandoffId ? layout.HandoffEquipment.GetComponentInChildren<Renderer>() : target.GetComponent<Renderer>();
            Require(model.TryGetSelectedBounds(out var bounds) && bounds.size.sqrMagnitude > 0 && bounds.Contains(highlighted.bounds.center), "Highlight bounds exclude equipment renderer.");
            targetMappings++;
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
