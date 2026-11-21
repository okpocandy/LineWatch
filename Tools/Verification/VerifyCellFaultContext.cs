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

public static class VerifyCellFaultContext
{
    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var cell = layout.GetComponent<CellRunController>(); var tracker = layout.GetComponent<CellProductTracker>();
        await Until(() => tracker.Count == 1, "Initial registration timeout.");
        Require(!cell.IsFaulted && !cell.ActiveFault.HasValue && !cell.LastAcknowledgedFault.HasValue, "Normal startup has an alarm.");
        string products = Products(); string productId = tracker.InputProductId;
        var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
        var box = window.rootVisualElement.Q<HelpBox>("cell-run-fault");
        Require(box != null && box.style.display.value == DisplayStyle.None, "Normal startup shows fault UI.");
        Require(cell.TryPrepareAutomaticTransfer(out var error) && cell.TryExecute(CellRunCommand.Start, out error), error);
        var sequence = layout.Scara.GetComponent<RobotWaypointSequence>();
        Require(sequence.State == RobotWaypointSequence.RunState.Running, "SCARA did not start.");
        layout.Scara.Axis1.enabled = false;
        await Until(() => cell.IsFaulted && cell.ActiveFault.HasValue, "SCARA fault was not observed.");
        var first = cell.ActiveFault.Value;
        Require(first.Source == CellRunController.FaultSource.Scara && first.EquipmentId == layout.ScaraId && first.OperationProductId == productId &&
            first.Step == sequence.StepName && first.Reason == cell.FaultReason, "First fault context mismatch.");
        Require(cell.TryExecute(CellRunCommand.Stop, out error) && cell.ActiveFault.Value.Equals(first) && cell.IsFaulted, error ?? "Stop cleared fault context.");
        Require(!cell.TryExecute(CellRunCommand.Reset, out _) && cell.ActiveFault.Value.Equals(first), "Invalid recovery cleared context.");
        layout.Scara.Axis1.enabled = true;
        tracker.enabled = false; tracker.enabled = true;
        await Task.Delay(80);
        Require(cell.IsFaulted && cell.ActiveFault.Value.Equals(first), "Restored hardware/second fault overwrote first context.");
        await Until(() => box.text.Contains(first.EquipmentId) && box.text.Contains(first.OperationProductId) && box.text.Contains(first.Step), "Fault UI context missing.");
        Require(Products() == products, "Fault handling changed products, ownership, FIFO or slots.");
        Require(cell.TryExecute(CellRunCommand.Reset, out error), error);
        Require(cell.State == CellRunState.Stopped && !cell.ActiveFault.HasValue && cell.LastAcknowledgedFault.HasValue && cell.LastAcknowledgedFault.Value.Equals(first), "Acknowledgement lost fault history.");
        await Until(() => box.text.Contains("최근 명시 해제") && box.text.Contains(first.EquipmentId), "Acknowledged UI context missing.");
        Require(cell.State == CellRunState.Stopped && !layout.Scara.IsTrajectoryMoving && !layout.Fanuc.IsTcpMoving && Products() == products, "Acknowledgement restarted or changed products.");
        layout.Fanuc.Joints[0].enabled = false;
        await Until(() => cell.IsFaulted && cell.ActiveFault.HasValue, "FANUC wiring fault was not observed.");
        var fanucFault = cell.ActiveFault.Value;
        Require(fanucFault.Source == CellRunController.FaultSource.Fanuc && fanucFault.EquipmentId == layout.FanucId &&
            fanucFault.OperationProductId == null && fanucFault.Step == "대기" && cell.LastAcknowledgedFault.Value.Equals(first), "FANUC idle fault identity/history mismatch.");
        layout.Fanuc.Joints[0].enabled = true;
        await Task.Delay(80);
        Require(cell.IsFaulted && cell.ActiveFault.Value.Equals(fanucFault) && Products() == products, "FANUC restoration cleared alarm or changed products.");
        var report = new { passed = true, normalWithoutAlarm = true, faultSource = first.Source.ToString(), first.EquipmentId, first.OperationProductId, first.Step, first.Reason,
            stopAndRejectedResetPreserveContext = true, reenablingAndSecondFaultPreserveFirst = true, explicitResetArchivesContext = true, noAutomaticRestart = true,
            fanucIdleFaultIdentified = true, productStatePreserved = true, faultAndAcknowledgedUiRead = true, directClicks = 0, callbacksOrUiInputInjected = 0 };
        Directory.CreateDirectory("Logs/LW14"); File.WriteAllText("Logs/LW14/fault-context.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;

        string Products() => JsonConvert.SerializeObject(new { records = tracker.Snapshot(), fifo = tracker.WaitingSnapshot(),
            slots = new[] { tracker.InputProductId, tracker.ScaraProductId, tracker.HandoffProductId, tracker.FanucProductId },
            items = layout.GetComponentsInChildren<GrippableItem>(true).Select(i => new { i.ProductId, owner = i.Owner != null ? i.Owner.GetInstanceID() : 0,
                position = i.Body.position.ToString("R"), rotation = i.Body.rotation.ToString("R") }) });
    }
    private static async Task Until(Func<bool> predicate, string message)
    { var watch = Stopwatch.StartNew(); while (!predicate()) { Require(watch.Elapsed.TotalSeconds < 8, message); await Task.Delay(16); } }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
