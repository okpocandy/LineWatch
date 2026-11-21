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

public static class VerifySupplyProgress
{
    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var c = layout.GetComponent<CellRunController>(); var t = layout.GetComponent<CellProductTracker>();
        float originalScale = Time.timeScale; var originalPhysics = Physics.simulationMode;
        const string nextId = "SUPPLY-WATCH-000002";
        try
        {
            Time.timeScale = 5;
            await Until(() => t.Count == 1, "Initial registration timeout.");
            var next = UnityEngine.Object.Instantiate(layout.ProductPrefab);
            next.Body.position = layout.InputStop.TransformPoint(new Vector3(0, 0, 2)); next.Body.rotation = layout.InputStop.rotation;
            Require(t.TryRegister(next, nextId, ProductDestination.B, out var error), error);
            Require(c.TryPrepareContinuousTransfer(out error) && c.TryExecute(CellRunCommand.Start, out error), error);
            await Until(() => c.ProgressKind == CellRunController.AutomaticProgressKind.NormalInputSupply && c.ProgressProductId == nextId, "Supply phase timeout.");
            float initialDistance = Vector3.Distance(next.Body.position, layout.InputStop.position);
            await Until(() => c.ProgressActiveSeconds > CellRunController.SupplyNoProgressTimeoutSeconds + .2f, "Normal supply observation timeout.");
            Require(!c.IsFaulted && c.SupplyNoProgressSeconds < CellRunController.SupplyNoProgressTimeoutSeconds &&
                Vector3.Distance(next.Body.position, layout.InputStop.position) < initialDistance - .1f, "Long normal motion was mistaken for a jam.");
            double normalSeconds = c.ProgressActiveSeconds;
            // Real Rigidbody progress stops while Update/FixedUpdate still run; no product state or UI callbacks are injected.
            Physics.simulationMode = SimulationMode.Script;
            await Until(() => c.SupplyNoProgressSeconds > .2, "Physics stall did not stop actual body progress.");
            string before = Products(); var stoppedPose = next.Body.position;
            int frozenDelay = (int)((CellRunController.SupplyNoProgressTimeoutSeconds + .5f) / Time.timeScale * 1000);
            Require(c.TryExecute(CellRunCommand.Pause, out error), error);
            double paused = c.SupplyNoProgressSeconds;
            await Task.Delay(frozenDelay);
            Require(c.SupplyNoProgressSeconds == paused && !c.IsFaulted, "Pause was counted as supply stagnation.");
            Require(c.TryExecute(CellRunCommand.Resume, out error), error);
            await Until(() => c.SupplyNoProgressSeconds > paused, "Resume did not continue stagnation clock.");
            Require(c.TryExecute(CellRunCommand.Stop, out error), error);
            double stopped = c.SupplyNoProgressSeconds;
            await Task.Delay(frozenDelay);
            Require(c.SupplyNoProgressSeconds == stopped && !c.IsFaulted, "Stop was counted as supply stagnation.");
            Require(c.TryExecute(CellRunCommand.Start, out error), error);
            await Until(() => c.IsFaulted, "Supply stall did not fault.");
            Require(c.ActiveFault.HasValue, "Supply alarm missing.");
            var fault = c.ActiveFault.Value;
            Require(fault.Source == CellRunController.FaultSource.InputSupply && fault.OperationProductId == nextId && fault.Step == "선두 상품 공급 중" &&
                c.SupplyNoProgressSeconds >= CellRunController.SupplyNoProgressTimeoutSeconds && Products() == before && next.Body.position == stoppedPose,
                "Supply alarm context/conservation mismatch.");
            var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
            Require(window.rootVisualElement.Q<HelpBox>("cell-run-fault").text.Contains("투입 공급") &&
                window.rootVisualElement.Q<Label>("cell-run-progress").text.Contains("공급 중단"), "Supply fault UI mismatch.");
            double faultSeconds = c.SupplyNoProgressSeconds;
            Physics.simulationMode = originalPhysics;
            await Task.Delay(frozenDelay);
            Require(c.IsFaulted && c.ActiveFault.Value.Equals(fault) && c.SupplyNoProgressSeconds == faultSeconds && Products() == before, "Physics restoration cleared or resumed the fault.");
            Require(c.TryExecute(CellRunCommand.Reset, out error), error);
            Require(c.State == CellRunState.Stopped && c.SupplyNoProgressSeconds == 0 && !c.ActiveFault.HasValue && c.LastAcknowledgedFault.Value.Equals(fault), "Explicit reset did not rearm the supply clock/history.");
            await Task.Delay(80);
            Require(c.State == CellRunState.Stopped && Products() == before, "Reset automatically restarted supply.");
            Require(c.TryExecute(CellRunCommand.Start, out error), error);
            await Until(() => c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Completed, "Recovered two-product completion timeout.");
            Require(!c.IsFaulted && t.TryGet(nextId, out var record) && record.Location == ProductLocation.OutputB && t.WaitingSnapshot().Count == 0, "Recovered supply failed.");
            var report = new { passed = true, productsUsed = 2, normalSupplySeconds = normalSeconds, longNormalMotionNotFaulted = true,
                stalledActiveSeconds = faultSeconds, thresholdSeconds = CellRunController.SupplyNoProgressTimeoutSeconds,
                pauseAndStopExcluded = true, source = fault.Source.ToString(), fault.OperationProductId, fault.Step,
                conservationPreserved = true, physicsRestorationDoesNotClearFault = true, explicitResetRearmsWithoutRestart = true,
                separateStartCompletes = true, faultUiRead = true, directClicks = 0, callbacksOrUiInputInjected = 0 };
            Directory.CreateDirectory("Logs/LW14"); File.WriteAllText("Logs/LW14/supply-progress.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;

            string Products() => JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(),
                slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId }, next.ProductId,
                owner = next.Owner != null ? next.Owner.GetInstanceID() : 0 });
        }
        finally { Physics.simulationMode = originalPhysics; Time.timeScale = originalScale; c.TryExecute(CellRunCommand.Stop, out _); }
    }
    private static async Task Until(Func<bool> predicate, string message)
    { var watch = Stopwatch.StartNew(); while (!predicate()) { Require(watch.Elapsed.TotalSeconds < 15, message); await Task.Delay(1); } }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
