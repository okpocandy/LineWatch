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

public static class VerifyEquipmentFraming
{
    public static async Task<object> Run()
    {
        Require(!EditorApplication.isPlaying, "This focused check uses Edit mode only.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        var scene = SceneView.lastActiveSceneView;
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity" && scene != null, "Need LineWatch and an existing Scene view.");
        var oldPivot = scene.pivot; var oldRotation = scene.rotation; var oldSize = scene.size; bool oldOrtho = scene.orthographic;
        var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
        var selection = window.EquipmentSelection; string oldSelection = selection.SelectedId;
        string before = Capture();
        try
        {
            selection.ClearSelection();
            var button = window.rootVisualElement.Q<Button>("equipment-frame-selected");
            Require(button != null && button.text.Contains("선택 설비 보기"), "Frame command UI missing.");
            await WaitFor(() => !button.enabledSelf, "Unselected frame button did not disable.");
            Require(!window.TryFrameSelectedEquipment(out var error) && !string.IsNullOrEmpty(error), "No selection was not rejected.");
            Require(scene.pivot == oldPivot && scene.rotation == oldRotation && scene.size == oldSize, "Rejected frame changed view.");
            Require(selection.TrySelect(layout.ScaraId, out error) && selection.TryGetSelectedBounds(out _), error ?? "No SCARA bounds.");
            await WaitFor(() => button.enabledSelf, "Selected frame button did not enable.");
            Require(scene.pivot == oldPivot && scene.rotation == oldRotation && scene.size == oldSize, "Selecting alone moved view.");
            selection.TryGetSelectedBounds(out var bounds);
            scene.pivot = bounds.center + Vector3.one * 2f;
            Require(window.TryFrameSelectedEquipment(out error), error);
            await WaitFor(() => Vector3.Distance(scene.pivot, bounds.center) < .0001f && scene.size > 0 && !float.IsInfinity(scene.size), "View did not reach selected bounds center.");
            Require(Quaternion.Angle(scene.rotation, oldRotation) < .01f && scene.orthographic == oldOrtho, "Framing changed viewing direction/projection.");
            Require(Capture() == before && selection.SelectedId == layout.ScaraId, "Frame changed scene, equipment, products or selection.");
        }
        finally
        {
            scene.pivot = oldPivot; scene.rotation = oldRotation; scene.size = oldSize; scene.orthographic = oldOrtho; scene.Repaint();
            selection.ClearSelection();
            if (oldSelection != null) selection.TrySelect(oldSelection, out _);
            window.rootVisualElement.Q<CellEquipmentView>()?.Refresh();
        }
        Require(Vector3.Distance(scene.pivot, oldPivot) < .0001f && Quaternion.Angle(scene.rotation, oldRotation) < .01f && Mathf.Abs(scene.size - oldSize) < .0001f, "Original Scene view was not restored.");
        var report = new { passed = true, normalFrameFlow = 1, unselectedRejected = true, buttonEligibilityRead = true, selectionAlonePreservesView = true,
            animatedFrameReachedCenter = true, originalSceneViewRestored = true, sceneAndGameCameraPreserved = true, mode = "Edit", directClicks = 0, callbacksOrUiInputInjected = 0 };
        Directory.CreateDirectory("Logs/LW13"); File.WriteAllText("Logs/LW13/equipment-framing.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;

        string Capture() => JsonConvert.SerializeObject(new
        {
            layout.gameObject.scene.isDirty,
            transforms = layout.gameObject.scene.GetRootGameObjects().SelectMany(r => r.GetComponentsInChildren<Transform>(true)).Select(t => new
            { id = t.GetInstanceID(), p = new[] { t.localPosition.x, t.localPosition.y, t.localPosition.z }, q = new[] { t.localRotation.x, t.localRotation.y, t.localRotation.z, t.localRotation.w }, scale = new[] { t.localScale.x, t.localScale.y, t.localScale.z } }),
            cell = layout.GetComponent<CellRunController>().State,
            records = layout.GetComponent<CellProductTracker>().Snapshot(), fifo = layout.GetComponent<CellProductTracker>().WaitingSnapshot(),
            scaraCommands = layout.Scara.CommandVersion, fanucCommands = layout.Fanuc.CommandVersion,
            products = layout.GetComponentsInChildren<GrippableItem>(true).Select(i => new { i.ProductId, owner = i.Owner != null ? i.Owner.GetInstanceID() : 0 })
        });
    }
    private static async Task WaitFor(Func<bool> predicate, string message)
    {
        var watch = Stopwatch.StartNew();
        while (!predicate()) { Require(watch.Elapsed.TotalSeconds < 5, message); await Task.Delay(16); }
    }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
