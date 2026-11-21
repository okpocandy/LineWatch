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

public static class VerifyRobotProgress
{
    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var c = layout.GetComponent<CellRunController>(); var t = layout.GetComponent<CellProductTracker>();
        float scale = Time.timeScale, fixedStep = Time.fixedDeltaTime, maximumStep = Time.maximumDeltaTime;
        double scaraSeconds = 0, fanucSeconds = 0;
        try
        {
            Time.timeScale = 5;
            await Until(() => t.Count == 1, "Initial registration timeout.");
            string id = t.InputProductId;
            Require(c.TryPrepareAutomaticTransfer(out var error) && c.TryExecute(CellRunCommand.Start, out error), error);
            await CheckRobot(true);
            await CheckRobot(false);
            await Until(() => c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Completed && c.ProgressKind == CellRunController.AutomaticProgressKind.None, "Recovered product completion timeout.");
            Require(!c.IsFaulted && c.State == CellRunState.Stopped && t.Count == 1 && t.TryGet(id, out var record) && record.Location == ProductLocation.OutputA &&
                t.InputProductId == null && t.ScaraProductId == null && t.HandoffProductId == null && t.FanucProductId == null, "Recovered product conservation mismatch.");
            var report = new { passed = true, productsUsed = 1, scaraStalledActiveSeconds = scaraSeconds, fanucStalledActiveSeconds = fanucSeconds,
                thresholdSeconds = CellRunController.RobotNoProgressTimeoutSeconds, actualJointAndProductStateFrozenDuringStall = true,
                pauseAndStopExcluded = true, contextAndFaultUiRead = true, restoredTickDoesNotClearFault = true,
                explicitResetWithoutRestart = true, separateStartCompletesOriginalProduct = true, directClicks = 0, callbacksOrUiInputInjected = 0 };
            Directory.CreateDirectory("Logs/LW14"); File.WriteAllText("Logs/LW14/robot-progress.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;

            async Task CheckRobot(bool isScara)
            {
                string equipment = isScara ? layout.ScaraId : layout.FanucId;
                await Until(() => c.ProgressEquipmentId == equipment && (isScara ? layout.Scara.IsTrajectoryMoving && layout.Scara.AppliedMotionSeconds > 0
                    : layout.Fanuc.IsTcpMoving && layout.Fanuc.AppliedMotionSeconds > 0), equipment + " normal motion timeout.");
                Require(!c.IsFaulted, "Normal applied motion faulted.");
                // Prevent fixed ticks while Update and the cell clock keep running; no motion callback is invoked.
                Time.fixedDeltaTime = 100;
                await Until(() => c.RobotNoProgressSeconds > .1, "Missing fixed tick did not stop applied motion.");
                string before = Products(), joints = Joints();
                if (isScara)
                {
                    int delay = (int)((CellRunController.RobotNoProgressTimeoutSeconds + .5f) / Time.timeScale * 1000);
                    Require(c.TryExecute(CellRunCommand.Pause, out var e), e);
                    double paused = c.RobotNoProgressSeconds; await Task.Delay(delay);
                    Require(c.RobotNoProgressSeconds == paused && !c.IsFaulted, "Pause counted as robot stagnation.");
                    Require(c.TryExecute(CellRunCommand.Resume, out e), e);
                    await Until(() => c.RobotNoProgressSeconds > paused, "Resume erased or failed to continue stalled time.");
                    Require(c.TryExecute(CellRunCommand.Stop, out e), e);
                    double stopped = c.RobotNoProgressSeconds; await Task.Delay(delay);
                    Require(c.RobotNoProgressSeconds == stopped && !c.IsFaulted, "Stop counted as robot stagnation.");
                    Require(c.TryExecute(CellRunCommand.Start, out e), e);
                }
                await Until(() => c.IsFaulted, equipment + " did not fault for stalled motion.");
                var fault = c.ActiveFault.Value;
                var sequence = isScara ? layout.Scara.GetComponent<RobotWaypointSequence>() : layout.Fanuc.GetComponent<RobotWaypointSequence>();
                Require(fault.Source == (isScara ? CellRunController.FaultSource.Scara : CellRunController.FaultSource.Fanuc) && fault.EquipmentId == equipment &&
                    fault.OperationProductId == id && fault.Step == sequence.StepName && c.RobotNoProgressSeconds >= CellRunController.RobotNoProgressTimeoutSeconds &&
                    Products() == before && Joints() == joints, "Robot stall context or physical state mismatch.");
                if (isScara) scaraSeconds = c.RobotNoProgressSeconds; else fanucSeconds = c.RobotNoProgressSeconds;
                var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
                Require(window.rootVisualElement.Q<HelpBox>("cell-run-fault").text.Contains(equipment) &&
                    window.rootVisualElement.Q<Label>("cell-run-progress").text.Contains("로봇 이동 진행 없음"), "Robot fault UI missing.");
                Time.fixedDeltaTime = fixedStep; Time.maximumDeltaTime = maximumStep;
                await Task.Delay(80);
                Require(c.IsFaulted && c.ActiveFault.Value.Equals(fault) && Products() == before, "Tick restoration cleared or resumed the fault.");
                Require(c.TryExecute(CellRunCommand.Reset, out var error), error);
                Require(c.State == CellRunState.Stopped && c.RobotNoProgressSeconds == 0 && c.LastAcknowledgedFault.Value.Equals(fault), "Reset lost history or failed to rearm.");
                await Task.Delay(80);
                Require(c.State == CellRunState.Stopped && Products() == before, "Reset automatically restarted robot.");
                Require(c.TryExecute(CellRunCommand.Start, out error), error);
            }
            string Joints() => JsonConvert.SerializeObject(new { scara = new[] { layout.Scara.Axis1.CurrentPosition, layout.Scara.Axis2.CurrentPosition,
                layout.Scara.Axis3.CurrentPosition, layout.Scara.Axis4.CurrentPosition }, fanuc = layout.Fanuc.Joints.Select(j => j.CurrentPosition) });
            string Products() => JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(),
                slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
                items = layout.GetComponentsInChildren<GrippableItem>(true).Select(i => new { i.ProductId, owner = i.Owner != null ? i.Owner.GetInstanceID() : 0,
                    position = i.Body.position.ToString("R"), rotation = i.Body.rotation.ToString("R") }) });
        }
        finally { Time.fixedDeltaTime = fixedStep; Time.maximumDeltaTime = maximumStep; Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
    }
    private static async Task Until(Func<bool> predicate, string message)
    { var watch = Stopwatch.StartNew(); while (!predicate()) { Require(watch.Elapsed.TotalSeconds < 15, message); await Task.Delay(1); } }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
