using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using State = LineWatch.RobotWaypointSequence.RunState;

// One normal integration case. Registration and completed-object clearance are fixture work;
// supply, gripping, handoff, destination selection and completion use the production controller.
public static class VerifyContinuousHundred
{
    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh unpaused Play.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sq = l.Scara.GetComponent<RobotWaypointSequence>(); var fq = l.Fanuc.GetComponent<RobotWaypointSequence>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var fg = l.Fanuc.GetComponent<RobotGripper>();
        var ids = new List<string>(); var items = new Dictionary<string, GrippableItem>();
        var rows = new List<object>(); float originalScale = Time.timeScale;
        int scaraGrips = 0, scaraReleases = 0, fanucGrips = 0, fanucReleases = 0, cleared = 0;
        float maxPosition = 0, maxRotation = 0; var watch = Stopwatch.StartNew();
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1);
            string initial = t.InputProductId;
            Require(t.TryGet(initial, out var initialRecord) && initialRecord.Destination == ProductDestination.A, "Initial A sample missing.");
            Require(t.TryGetItem(initial, out var first), "Initial original missing."); ids.Add(initial); items.Add(initial, first);
            float spacing = Mathf.Max(.025f, first.GetComponent<BoxCollider>().size.magnitude + .004f);
            // Near-first ordering leaves an empty approach to Input and avoids crossing the remaining staging grid.
            var offsets = Enumerable.Range(0, 100).Select(i => new Vector3((i % 10 - 4.5f) * spacing, 0, .06f + i / 10 * spacing))
                .OrderBy(p => p.sqrMagnitude).Take(99).ToArray();
            for (int i = 1; i < 100; i++)
            {
                string id = "INTEGRATION-" + (i + 1).ToString("000000");
                var item = UnityEngine.Object.Instantiate(l.ProductPrefab);
                item.Body.position = l.InputStop.TransformPoint(offsets[i - 1]); item.Body.rotation = l.InputStop.rotation;
                Require(t.TryRegister(item, id, i % 2 == 0 ? ProductDestination.A : ProductDestination.B, out var error), error);
                ids.Add(id); items.Add(id, item);
            }
            Require(t.Count == 100 && t.WaitingSnapshot().SequenceEqual(ids.Skip(1)), "Registration/FIFO fixture mismatch.");
            Require(c.TryPrepareContinuousTransfer(initialRecord, out var e) && c.TryExecute(CellRunCommand.Start, out e), e);
            Progress("running");
            for (int i = 0; i < 100; i++)
            {
                string id = ids[i]; var item = items[id]; var destination = i % 2 == 0 ? ProductDestination.A : ProductDestination.B;
                await Until(() =>
                {
                    Healthy();
                    if (item.Owner == sg) Require(sg.HeldItem == item && t.ScaraProductId == id && t.InputProductId == null, "SCARA actual/logical ownership mismatch.");
                    if (fq.CycleItem == item && fq.IsBusy)
                        Require(sq.CycleItem == item && sq.State == State.Completed && sq.GripCount == 1 && sq.ReleaseCount == 1,
                            "FANUC entered before the original SCARA program completed.");
                    return sq.CycleItem == item && sq.State == State.Completed;
                });
                Require(sq.GripCount == 1 && sq.ReleaseCount == 1, "SCARA action evidence missing.");
                Require(l.Scara.TryReadFkDiagnostics(out var fk, out e), e);
                Require(Vector3.Distance(fk.ActualPositionMetres, l.ScaraHandoffWaypoints.Retreat.PositionMetres) <= .002f &&
                    fk.PositionErrorMetres <= .00001f && fk.YawErrorDegrees <= .005f, "SCARA actual retreat/FK mismatch.");
                scaraGrips += sq.GripCount; scaraReleases += sq.ReleaseCount;
                await Until(() =>
                {
                    Healthy();
                    if (item.Owner == fg) Require(fg.HeldItem == item && t.FanucProductId == id && t.HandoffProductId == null, "FANUC actual/logical ownership mismatch.");
                    return fq.CycleItem == item && fq.State == State.Completed;
                });
                var location = destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB;
                var output = destination == ProductDestination.A ? l.OutputA : l.OutputB;
                Require(t.TryGet(id, out var result) && result.Location == location && result.Destination == destination &&
                    t.TryGetItem(id, out var original) && ReferenceEquals(original, item) && item.Owner == null && fg.HeldItem == null &&
                    fq.GripCount == 1 && fq.ReleaseCount == 1, "Completion identity/destination/physical ownership/actions mismatch.");
                float position = Vector3.Distance(item.Body.position, output.position);
                float rotation = RobotGripper.RotationError(item.Body.rotation, output.rotation);
                Require(position <= .002f && rotation <= .5f, "Actual output pose is outside tolerance.");
                maxPosition = Mathf.Max(maxPosition, position); maxRotation = Mathf.Max(maxRotation, rotation);
                fanucGrips += fq.GripCount; fanucReleases += fq.ReleaseCount;
                Conservation(i + 1);
                rows.Add(new { cycle = i + 1, id, destination = destination.ToString(), scaraGrips = 1, scaraReleases = 1,
                    fanucGrips = 1, fanucReleases = 1, actualPositionErrorMetres = position, actualRotationErrorDegrees = rotation,
                    completed = i + 1, physicallyConfirmedResidual = 99 - i, unconfirmed = 0 });
                if (i < 99)
                {
                    UnityEngine.Object.Destroy(item.gameObject); cleared++;
                }
                if ((i + 1) % 10 == 0) Progress("running");
            }
            Healthy(); Conservation(100);
            var final = t.Snapshot();
            Require(c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Completed && c.State == CellRunState.Stopped &&
                final.Count(r => r.Location == ProductLocation.OutputA) == 50 && final.Count(r => r.Location == ProductLocation.OutputB) == 50 &&
                scaraGrips == 100 && scaraReleases == 100 && fanucGrips == 100 && fanucReleases == 100 &&
                sg.HeldItem == null && fg.HeldItem == null && t.WaitingSnapshot().Count == 0, "Final 100-product accounting/automatic stop mismatch.");
            var report = new { passed = true, normalIntegrationRuns = 1, registered = 100, completedA = 50, completedB = 50,
                scaraGrips, scaraReleases, fanucGrips, fanucReleases, explicitRemoved = 0, lost = 0, physicallyConfirmedResidual = 0,
                unconfirmed = 0, originalIdsAndFifoPreserved = true, automaticSupplyAndRobotDispatch = true, commonInitialStarts = 1,
                fixtureRegisteredAdditionalProducts = 99, fixtureClearedCompletedObjects = cleared, remainingCompletedObjects = 1,
                maxActualPositionErrorMetres = maxPosition, maxActualRotationErrorDegrees = maxRotation, elapsedSeconds = watch.Elapsed.TotalSeconds,
                directClicks = 0, callbacksOrUiInputInjected = 0, rows };
            Write("continuous-hundred.json", report); Progress("completed");
            return new { report.passed, report.registered, report.completedA, report.completedB, scaraGrips, scaraReleases, fanucGrips, fanucReleases,
                report.unconfirmed, report.maxActualPositionErrorMetres, report.maxActualRotationErrorDegrees, report.elapsedSeconds };
        }
        catch (Exception ex)
        {
            var current = c.AutomaticProductId;
            ProductRecord record = null; GrippableItem item = null;
            if (current != null) { t.TryGet(current, out record); t.TryGetItem(current, out item); }
            var output = record != null && record.Destination == ProductDestination.B ? l.OutputB : l.OutputA;
            Write("continuous-hundred-failed.json", new { passed = false, error = ex.Message, completedObserved = rows.Count, current,
                c.State, c.FaultReason, c.AutomaticPhase, trackerFault = t.FaultReason, record,
                scara = new { sq.State, sq.GripCount, sq.ReleaseCount, sq.LastError }, fanuc = new { fq.State, fq.GripCount, fq.ReleaseCount, fq.LastError },
                itemPosition = item != null ? (object)new[] { item.Body.position.x, item.Body.position.y, item.Body.position.z } : null,
                outputPosition = new[] { output.position.x, output.position.y, output.position.z },
                currentOutputErrorMetres = item != null ? (float?)Vector3.Distance(item.Body.position, output.position) : null,
                cleared, elapsedSeconds = watch.Elapsed.TotalSeconds, rows });
            Progress("failed"); throw;
        }
        finally { Time.timeScale = originalScale; c.TryExecute(CellRunCommand.Stop, out _); }

        void Healthy() => Require(!c.IsFaulted && !t.IsFaulted, c.FaultReason ?? t.FaultReason ?? "Cell faulted.");
        void Conservation(int complete)
        {
            var snapshot = t.Snapshot(); var active = snapshot.Where(r => r.Location != ProductLocation.Waiting && r.Location != ProductLocation.OutputA && r.Location != ProductLocation.OutputB).ToArray();
            Require(snapshot.Count() == 100 && snapshot.Count(r => r.Location == ProductLocation.OutputA || r.Location == ProductLocation.OutputB) == complete &&
                active.Length <= 1 && active.All(r => complete < 100 && r.Id == ids[complete]), "Conservation or active FIFO order mismatch.");
            Require(t.WaitingSnapshot().SequenceEqual(ids.Skip(complete + active.Length)), "Waiting FIFO order changed.");
            var slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId }.Where(id => id != null).ToArray();
            Require(slots.Length == active.Length && slots.Distinct().Count() == slots.Length && slots.SequenceEqual(active.Select(r => r.Id)), "Single occupancy mismatch.");
            foreach (var r in snapshot.Where(r => r.Location != ProductLocation.OutputA && r.Location != ProductLocation.OutputB))
            {
                Require(t.TryGetItem(r.Id, out var actual) && ReferenceEquals(actual, items[r.Id]) && actual.IsReady && actual.ProductTracker == t && actual.ProductId == r.Id,
                    "Nonterminal original product evidence missing.");
                var owner = r.Location == ProductLocation.Scara ? sg : r.Location == ProductLocation.Fanuc ? fg : null;
                Require(actual.Owner == owner && (owner == null || owner.HeldItem == actual), "Physical residual/holder evidence mismatch.");
                if (r.Location == ProductLocation.Input || r.Location == ProductLocation.Handoff)
                {
                    var station = r.Location == ProductLocation.Input ? l.InputStop : l.HandoffPoint;
                    Require(Vector3.Distance(actual.Body.position, station.position) <= .002f && RobotGripper.RotationError(actual.Body.rotation, station.rotation) <= .5f,
                        "Residual station evidence mismatch.");
                }
            }
        }
        void Progress(string state) => Write("continuous-hundred-progress.json", new { state, completedObserved = rows.Count, target = 100,
            current = c.AutomaticProductId, phase = c.AutomaticPhase.ToString(), elapsedSeconds = watch.Elapsed.TotalSeconds });
    }

    private static async Task Until(Func<bool> condition)
    {
        var watch = Stopwatch.StartNew();
        while (!condition()) { Require(watch.Elapsed.TotalSeconds < 25, "Per-phase verification wait timed out."); await Task.Delay(1); }
    }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
    private static void Write(string name, object report)
    { Directory.CreateDirectory("Logs/LW12A"); File.WriteAllText("Logs/LW12A/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
}
