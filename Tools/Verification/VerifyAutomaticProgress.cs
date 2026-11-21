using System;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

public static class VerifyAutomaticProgress
{
    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var c = layout.GetComponent<CellRunController>(); var t = layout.GetComponent<CellProductTracker>();
        float originalScale = Time.timeScale;
        const string nextId = "PROGRESS-000002";
        try
        {
            Time.timeScale = 5;
            await Until(() => t.Count == 1, "Initial registration timeout.");
            Require(c.ProgressKind == CellRunController.AutomaticProgressKind.None && c.ProgressActiveSeconds == 0, "Stopped startup is being timed.");
            var next = UnityEngine.Object.Instantiate(layout.ProductPrefab);
            next.Body.position = layout.InputStop.TransformPoint(new Vector3(0, 0, 1)); next.Body.rotation = layout.InputStop.rotation;
            Require(t.TryRegister(next, nextId, ProductDestination.B, out var error), error);
            Require(c.TryPrepareContinuousTransfer(out error) && c.TryExecute(CellRunCommand.Start, out error), error);
            await Until(() => c.ProgressKind == CellRunController.AutomaticProgressKind.RobotStep && c.ProgressEquipmentId == layout.ScaraId && c.ProgressActiveSeconds > 0, "SCARA timing missing.");
            Require(c.TryExecute(CellRunCommand.Pause, out error), error);
            double paused = c.ProgressActiveSeconds; string pausedProduct = c.ProgressProductId; string pausedStep = c.ProgressStep;
            await Task.Delay(100);
            Require(c.ProgressActiveSeconds == paused && c.ProgressProductId == pausedProduct && c.ProgressStep == pausedStep && !c.IsProgressClockRunning && !c.IsFaulted, "Pause changed timing context.");
            Require(c.TryExecute(CellRunCommand.Resume, out error), error);
            await Until(() => c.ProgressKind == CellRunController.AutomaticProgressKind.RobotStep && c.ProgressEquipmentId == layout.FanucId, "FANUC timing identity missing.");
            await Until(() => c.ProgressKind == CellRunController.AutomaticProgressKind.NormalInputSupply && c.ProgressProductId == nextId && c.ProgressActiveSeconds > 0, "Normal supply wait missing.");
            Require(c.ProgressEquipmentId == null && !c.IsFaulted && c.IsProgressClockRunning && t.WaitingSnapshot().Count == 1, "Supply was classified as a fault or robot step.");
            var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
            var label = window.rootVisualElement.Q<Label>("cell-run-progress");
            Require(label != null && label.text.Contains("정상 공급 대기") && label.text.Contains(nextId), "Supply UI state missing.");
            Require(c.TryExecute(CellRunCommand.Stop, out error), error);
            double stopped = c.ProgressActiveSeconds; var pose = next.Body.position;
            await Until(() => label.text.Contains("측정 동결"), "Stopped clock UI missing.");
            Require(c.ProgressActiveSeconds == stopped && !c.IsProgressClockRunning && next.Body.position == pose && t.WaitingSnapshot().Count == 1 && !c.IsFaulted, "Stop changed timing or supply state.");
            Require(c.TryExecute(CellRunCommand.Start, out error), error);
            await Until(() => c.ProgressActiveSeconds > stopped || c.ProgressKind != CellRunController.AutomaticProgressKind.NormalInputSupply, "Supply timing did not resume.");
            await Until(() => c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Completed && c.ProgressKind == CellRunController.AutomaticProgressKind.None, "Two-product completion/timing clear timeout.");
            Require(c.State == CellRunState.Stopped && !c.IsFaulted && c.ProgressActiveSeconds == 0 && t.TryGet(nextId, out var record) && record.Location == ProductLocation.OutputB &&
                t.WaitingSnapshot().Count == 0 && t.InputProductId == null && t.ScaraProductId == null && t.HandoffProductId == null && t.FanucProductId == null, "Completed operation or conservation mismatch.");
            var report = new { passed = true, productsUsed = 2, scaraAndFanucStepIdentity = true, pauseFreezesContextAndSeconds = true, normalSupplyNotFaulted = true,
                stopFreezesSupplyAndSeconds = true, resumePreservesElapsed = true, completedClearsClock = true, supplyAndFrozenUiRead = true,
                clockUnit = "scaled simulation seconds", automaticTimeoutImplemented = false, directClicks = 0, callbacksOrUiInputInjected = 0 };
            Directory.CreateDirectory("Logs/LW14"); File.WriteAllText("Logs/LW14/automatic-progress.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;
        }
        finally { Time.timeScale = originalScale; c.TryExecute(CellRunCommand.Stop, out _); }
    }
    private static async Task Until(Func<bool> predicate, string message)
    { var watch = Stopwatch.StartNew(); while (!predicate()) { Require(watch.Elapsed.TotalSeconds < 15, message); await Task.Delay(1); } }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
