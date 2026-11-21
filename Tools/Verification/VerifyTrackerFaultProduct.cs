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

public static class VerifyTrackerFaultProduct
{
    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var c = layout.GetComponent<CellRunController>(); var t = layout.GetComponent<CellProductTracker>();
        await Until(() => t.Count == 1, "Initial registration timeout.");
        string operationId = t.InputProductId;
        Require(t.TryGetItem(operationId, out var initial), "Initial product missing.");
        var initialPosition = initial.Body.position;
        const string otherId = "TRACKER-CAUSE-000002";
        var other = UnityEngine.Object.Instantiate(layout.ProductPrefab);
        other.Body.position = layout.InputStop.TransformPoint(new Vector3(0, 0, .2f)); other.Body.rotation = layout.InputStop.rotation;
        Require(t.TryRegister(other, otherId, ProductDestination.B, out var error) && c.TryPrepareAutomaticTransfer(out error), error);
        Require(!c.IsFaulted && !t.IsFaulted && t.FaultProductId == null, "Normal preparation has a tracker fault.");
        string before = Products();
        other.enabled = false; // Registered waiting B is unrelated to the prepared operation A.
        await Until(() => c.IsFaulted && t.IsFaulted, "Unavailable product event was not observed.");
        var first = c.ActiveFault.Value;
        Require(first.Source == CellRunController.FaultSource.ProductTracker && first.OperationProductId == operationId && first.CauseProductId == otherId &&
            t.FaultProductId == otherId && Products() == before, "Operation ID was substituted for the unavailable cause product.");
        ReadUi(otherId);
        Require(c.TryExecute(CellRunCommand.Stop, out error) && !c.TryExecute(CellRunCommand.Reset, out _), error ?? "Invalid reset accepted.");
        other.enabled = true;
        initial.Body.position = initialPosition + Vector3.right * .04f;
        t.enabled = false; t.enabled = true;
        await Task.Delay(80);
        Require(c.IsFaulted && t.IsFaulted && c.ActiveFault.Value.Equals(first) && t.FaultProductId == otherId, "Restoration/secondary fault overwrote original cause.");
        initial.Body.position = initialPosition;
        await Task.Delay(40);
        Reset(otherId);

        // Polling discovers the original Input product without an unavailable/owner event.
        initial.Body.position = initialPosition + Vector3.right * .04f;
        await Until(() => c.IsFaulted && t.IsFaulted, "Physical polling mismatch was not observed.");
        Require(c.ActiveFault.Value.CauseProductId == operationId && t.FaultProductId == operationId, "Physical polling lost the failing registry ID.");
        ReadUi(operationId);
        initial.Body.position = initialPosition;
        await Task.Delay(80);
        Require(c.IsFaulted && t.IsFaulted, "Restored pose automatically cleared fault.");
        Reset(operationId);

        t.enabled = false; t.enabled = true;
        await Until(() => c.IsFaulted && t.IsFaulted, "Tracker disable fault was not observed.");
        Require(c.ActiveFault.Value.Source == CellRunController.FaultSource.ProductTracker && c.ActiveFault.Value.OperationProductId == operationId &&
            c.ActiveFault.Value.CauseProductId == null && t.FaultProductId == null, "Generic tracker fault borrowed an operation product ID.");
        ReadUi(null);
        Reset(null);
        Require(Products() == before && c.State == CellRunState.Stopped && !layout.Scara.IsTrajectoryMoving && !layout.Fanuc.IsTcpMoving, "Diagnostic changes mutated product state or restarted robots.");
        var report = new { passed = true, productsUsed = 2, operationProductId = operationId, unavailableCauseProductId = otherId,
            operationAndCauseDistinguished = true, physicalPollingCauseRecorded = true, genericTrackerCauseRemainsUnknown = true,
            firstCausePreserved = true, explicitResetArchivesCause = true, restoredPoseDoesNotClearFault = true,
            recordsFifoSlotsOwnersPreserved = true, noAutomaticRestart = true, threeFaultUiReads = true, directClicks = 0, callbacksOrUiInputInjected = 0 };
        Directory.CreateDirectory("Logs/LW14"); File.WriteAllText("Logs/LW14/tracker-fault-product.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;

        void ReadUi(string cause)
        {
            var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
            var text = window.rootVisualElement.Q<HelpBox>("cell-run-fault").text;
            Require(text.Contains("운전 대상 상품: " + operationId) && text.Contains("원인 상품: " + (cause ?? "특정되지 않음")), "Cause/operation UI mismatch.");
        }
        void Reset(string cause)
        {
            Require(c.TryExecute(CellRunCommand.Reset, out var e), e);
            Require(!c.IsFaulted && !t.IsFaulted && !c.ActiveFault.HasValue && t.FaultProductId == null &&
                c.LastAcknowledgedFault.Value.CauseProductId == cause && t.LastAcknowledgedFaultProductId == cause && c.State == CellRunState.Stopped,
                "Reset did not archive/clear matching cause IDs.");
        }
        string Products() => JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(),
            slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            items = layout.GetComponentsInChildren<GrippableItem>(true).Select(i => new { i.ProductId, owner = i.Owner != null ? i.Owner.GetInstanceID() : 0 }) });
    }
    private static async Task Until(Func<bool> predicate, string message)
    { var watch = Stopwatch.StartNew(); while (!predicate()) { Require(watch.Elapsed.TotalSeconds < 8, message); await Task.Delay(1); } }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
