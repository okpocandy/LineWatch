using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using State = LineWatch.RobotWaypointSequence.RunState;

public static class VerifyScaraTransfer
{
    public static async Task<string> Normal(int interruption)
    {
        Require(interruption >= 0 && interruption <= 6, "Use normal or interruption 0..6.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = l.Scara.GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>();
        var fg = l.Fanuc.GetComponent<RobotGripper>(); var fq = fg.GetComponent<RobotWaypointSequence>();
        var originalDrop = l.ScaraHandoffWaypoints; RobotWaypointSet copy = null; var hashes = Sources(); float scale = Time.timeScale;
        int rejected = 0, heldSamples = 0; float maxHeldError = 0; string fanucBefore = FanucState(l);
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); Add(l, "WAIT-A", .4f); Add(l, "WAIT-B", .8f);
            Require(t.TryGetItem("P-000001", out var item), "Input missing.");
            if (interruption == 6) { copy = UnityEngine.Object.Instantiate(originalDrop); SetDrop(l, copy); }
            string products = Products(l), physical = Physical(l), motion = Motion(l), before = Full(l);
            for (int i = 0; i < 100; i++) Require(c.CanPrepareScaraTransfer(out var e), e);
            Require(Full(l) == before, "Preparation queries mutated evidence.");
            Require(c.TryPrepareScaraTransfer(out var error), error);
            Require(c.State == CellRunState.Stopped && c.HasControlledSequence && c.ControlledSequence == q && q.State == State.Paused &&
                q.IsCellTransfer && q.IsPickPlace && q.StepCount == 7 && q.StepIndex == 0 && q.GripCount == 0 && q.ReleaseCount == 0 &&
                Products(l) == products && Physical(l) == physical && Motion(l) == motion, "Preparation moved, changed product data or dispatched a command.");
            before = Full(l); await Ticks(3); Require(Full(l) == before, "Preparation automatically started on later frames.");
            Reject(() => c.TryPrepareScaraTransfer(out _), "duplicate preparation");
            Reject(() => q.TryStart(out _), "individual program replacement");
            Reject(() => g.TryGrip(item, out _), "manual grip");
            Reject(() => fq.TryStart(out _), "other robot start");
            Require(c.TryExecute(CellRunCommand.Start, out error), error);
            Reject(() => c.TryExecute(CellRunCommand.Start, out _), "duplicate start");
            if (interruption != 0)
            {
                if (interruption == 1) c.TryExecute(CellRunCommand.Pause, out _);
                else
                {
                    await Until(() => q.ReleaseCount == 1 || q.GripCount == 1 && interruption != 3 || c.IsFaulted);
                    Require(!c.IsFaulted, q.LastError ?? c.FaultReason);
                    if (interruption == 3) Require(q.ReleaseCount == 1, "Post-release pause fixture missed release.");
                    if (interruption == 4) Require(c.TryExecute(CellRunCommand.Stop, out error), error);
                    else if (interruption == 5) { c.enabled = false; c.enabled = true; }
                    else if (interruption == 6)
                    {
                        var retreat = copy.Retreat.PositionMetres; SetRetreat(copy, retreat + Vector3.right * .01f);
                        await Until(() => q.State == State.Faulted && c.IsFaulted); SetRetreat(copy, retreat);
                    }
                    else Require(c.TryExecute(CellRunCommand.Pause, out error), error);
                }
                await Ticks(2);
                string data = Data(l); string stable = Full(l); await Ticks(3); Require(Full(l) == stable, "Interruption progressed or lost evidence.");
                Reject(() => c.TryPrepareScaraTransfer(out _), "prepare while pending");
                Reject(() => g.TryRelease(out _), "manual release while owned");
                Reject(() => q.TryResume(out _), "individual resume while owned");
                var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show(); w.RefreshCell();
                Require(w.rootVisualElement.Q<Label>("cell-run-scara").text.Contains("/7"), "UI reports the old four-step limit for a transfer.");
                if (interruption >= 5)
                {
                    Require(c.IsFaulted && (interruption != 6 || q.State == State.Faulted), "Expected latched fault.");
                    Require(c.TryExecute(CellRunCommand.Reset, out error), error);
                    Require(c.State == CellRunState.Stopped && q.State == State.Paused && Data(l) == data, "Acknowledgement changed holding/steps/product evidence.");
                    stable = Full(l); await Ticks(2); Require(Full(l) == stable, "Acknowledgement automatically resumed.");
                }
                Require(c.TryExecute(c.State == CellRunState.Paused ? CellRunCommand.Resume : CellRunCommand.Start, out error), error);
            }
            await Until(() =>
            {
                if (item.Owner == g)
                {
                    Require(t.ScaraProductId == item.ProductId && t.HandoffProductId == null && g.HeldItem == item, "Actual holding and logical slot diverged.");
                    if (g.Scara.TryReadFkDiagnostics(out var d, out _)) { heldSamples++; maxHeldError = Mathf.Max(maxHeldError, d.PositionErrorMetres); }
                }
                return q.State == State.Completed || c.IsFaulted;
            });
            Require(q.State == State.Completed && q.StepIndex == 7 && q.LastArrivedStepIndex == 6 && q.GripCount == 1 && q.ReleaseCount == 1 &&
                t.TryGet(item.ProductId, out var result) && result.Location == ProductLocation.Handoff && result.Destination == ProductDestination.A &&
                t.Count == 3 && t.InputProductId == null && t.ScaraProductId == null && t.HandoffProductId == "P-000001" &&
                t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B" }) && item.Owner == null && g.HeldItem == null && fg.HeldItem == null &&
                fq.State == State.Idle && FanucState(l) == fanucBefore && !t.IsFaulted && !c.IsFaulted,
                q.LastError ?? c.FaultReason ?? "Transfer failed or repeated an action/changed identity/FIFO/other robot.");
            float placement = Vector3.Distance(item.Body.position, l.HandoffPoint.position);
            float rotation = RobotGripper.RotationError(item.Body.rotation, l.HandoffPoint.rotation);
            Require(placement <= RobotGripper.PositionToleranceMetres && rotation <= RobotGripper.RotationToleranceDegrees, "Physical handoff pose is outside tolerance.");
            Require(l.Scara.TryReadFkDiagnostics(out var fk, out error), error);
            Require(Vector3.Distance(fk.ActualPositionMetres, originalDrop.Retreat.PositionMetres) <= .002f, "SCARA did not retreat after release.");
            Require(c.TryExecute(CellRunCommand.Stop, out error), error);
            motion = Motion(l); products = Products(l);
            Require(c.TryExecute(CellRunCommand.Start, out error), error);
            await Ticks(2); Require(Motion(l) == motion && Products(l) == products && q.GripCount == 1 && q.ReleaseCount == 1, "Completed program was automatically replayed.");
            Require(c.TryExecute(CellRunCommand.Stop, out error) && c.TryReleaseControl(out error), error);
            Reject(() => c.TryPrepareScaraTransfer(out _), "occupied handoff with completed input");
            Check(hashes);
            var report = new { passed = true, interruption, preparedWithoutMotion = true, completedPrograms = 1, automaticGrips = q.GripCount,
                automaticReleases = q.ReleaseCount, registeredProducts = t.Count, physicallyConfirmedResidual = 3, unconfirmedProducts = 0,
                waitingOrderPreserved = true, handoffProduct = t.HandoffProductId, destinationPreserved = true, finalStep = q.StepIndex,
                placementErrorMetres = placement, placementRotationErrorDegrees = rotation, heldFkSamples = heldSamples, maxHeldFkErrorMetres = heldSamples > 0 ? (float?)maxHeldError : null,
                preparationQueriesPreserved = 100, rejectedRequestsPreserved = rejected, fanucJointStatePreserved = FanucState(l) == fanucBefore, fanucCommandVersionIncludingStops = l.Fanuc.CommandVersion,
                directClicks = 0, callbacksOrUiInputInjected = 0, sourcesPreserved = hashes.Count, hashes };
            Write("scara-transfer-normal-" + interruption + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, interruption, report.automaticGrips, report.automaticReleases, report.placementErrorMetres, report.rejectedRequestsPreserved });
        }
        finally { c.enabled = true; Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetDrop(l, originalDrop); if (copy != null) UnityEngine.Object.Destroy(copy); }
        void Reject(Func<bool> action, string name)
        { string before = Full(l); Require(!action() && Full(l) == before, "Rejected request changed evidence: " + name); rejected++; }
    }

    public static async Task<string> Blocked(int scenario)
    {
        Require(scenario >= 0 && scenario <= 5, "Use blocked scenario 0..5.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = l.Scara.GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>();
        var originalDrop = l.ScaraHandoffWaypoints; var copy = UnityEngine.Object.Instantiate(originalDrop); var hashes = Sources();
        float scale = Time.timeScale; int rejected = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); Require(t.TryGetItem("P-000001", out var item), "Input missing.");
            SetDrop(l, copy);
            if (scenario <= 1)
            {
                SetRetreat(copy, new Vector3(9, copy.Retreat.PositionMetres.y, 9));
                if (scenario == 1) { var so = new SerializedObject(copy); so.FindProperty("work").FindPropertyRelative("positionMetres").vector3Value += Vector3.right * .02f; so.ApplyModifiedPropertiesWithoutUndo(); }
                Reject("invalid final IK or placement");
                Require(q.State == State.Idle && q.StepCount == 4 && !q.IsCellTransfer && !c.HasControlledSequence, "Rejected preparation partially installed a program.");
            }
            else if (scenario == 2)
            {
                var fg = l.Fanuc.GetComponent<RobotGripper>(); var lab = fg.GetComponent<RobotGripperLab>();
                Require(lab.TryPrepare(out var e) && lab.TryMoveToWork(out e), e); await Until(() => fg.IsStopped);
                Require(fg.TryGrip(lab.TrialItem, out e), e); Reject("other actual holding");
            }
            else if (scenario == 3)
            {
                var fq = l.Fanuc.GetComponent<RobotWaypointSequence>(); Require(fq.TryStart(out var e), e);
                Reject("other program running"); fq.Pause(); Reject("other program pending");
            }
            else if (scenario == 4)
            {
                Time.timeScale = 0; Reject("time stopped"); Time.timeScale = 5;
                EditorApplication.isPaused = true; Reject("Editor paused"); EditorApplication.isPaused = false;
                c.enabled = false; Reject("manager disabled"); c.enabled = true; Reject("reactivation still faulted");
                Require(c.TryExecute(CellRunCommand.Reset, out var e), e); Time.timeScale = 1;
                Require(c.TryPrepareScaraTransfer(out e), e);
                var point = item.GetGripPoint(RobotWaypointSet.RobotType.Scara); var position = point.localPosition;
                Require(c.TryExecute(CellRunCommand.Start, out e), e); point.localPosition += Vector3.up * .02f;
                await Until(() => q.State == State.Faulted && c.IsFaulted);
                Require(q.GripCount == 0 && q.ReleaseCount == 0 && item.Owner == null && t.InputProductId == item.ProductId && !t.IsFaulted,
                    "Failed grip changed product/physical ownership or action count.");
                string before = Full(l); Require(!c.TryExecute(CellRunCommand.Reset, out _) && Full(l) == before, "Invalid planned grip was acknowledged."); rejected++;
                point.localPosition = position; await Ticks(2); Require(c.IsFaulted, "Grip-point repair automatically cleared a fault.");
                Require(c.TryExecute(CellRunCommand.Reset, out e) && c.TryExecute(CellRunCommand.Start, out e), e);
                await Until(() => q.State == State.Completed || c.IsFaulted);
                Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1 && t.HandoffProductId == item.ProductId, q.LastError ?? c.FaultReason);
            }
            else
            {
                await Move(g, item, l.InputStop); Require(g.TryGrip(item, out var e), e);
                await Move(g, item, l.HandoffPoint); Require(g.TryRelease(out e), e);
                var next = UnityEngine.Object.Instantiate(l.ProductPrefab); next.Body.position = l.InputStop.position; next.Body.rotation = l.InputStop.rotation;
                Require(t.TryRegister(next, "NEXT", ProductDestination.B, out e) && t.TryFeedNext(out _, out e), e);
                Reject("occupied handoff with a different valid input");
            }
            Check(hashes); var report = new { passed = true, scenario, rejectedRequestsPreserved = rejected,
                noPartialPreparation = scenario <= 1, failedGripThenExplicitRecovery = scenario == 4,
                directClicks = 0, callbacksOrUiInputInjected = 0, sourcesPreserved = hashes.Count, hashes };
            Write("scara-transfer-blocked-" + scenario + ".json", report); return JsonConvert.SerializeObject(new { report.passed, scenario, report.rejectedRequestsPreserved });
        }
        finally { EditorApplication.isPaused = false; c.enabled = true; Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetDrop(l, originalDrop); UnityEngine.Object.Destroy(copy); }
        void Reject(string name) { string before = Full(l); Require(!c.TryPrepareScaraTransfer(out var e) && !string.IsNullOrEmpty(e) && Full(l) == before, "Rejected preparation mutated evidence: " + name); rejected++; }
    }

    public static async Task<string> Lifecycle()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var q = l.Scara.GetComponent<RobotWaypointSequence>(); var hashes = Sources(); float scale = Time.timeScale;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1);
            Require(c.TryPrepareScaraTransfer(out var e) && c.TryExecute(CellRunCommand.Start, out e), e);
            await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1, q.LastError ?? c.FaultReason);
            Require(c.TryExecute(CellRunCommand.Stop, out e) && c.TryReleaseControl(out e), e);
            Require(!c.TryResetScenario("인계 잔류를 숨기면 안 됨", out _), "Scenario reset hid an active handoff product.");
            Require(t.TryRemove("P-000001", ProductLocation.Handoff, out e) && c.TryResetScenario("SCARA 운반 완료 이력 보존", out e), e);
            var archived = q.ScenarioResetSnapshot().Last();
            Require(archived.IsCellTransfer && archived.Targets.Count == 7 && archived.GripCount == 1 && archived.ReleaseCount == 1 &&
                q.State == State.Idle && q.StepCount == 4 && !q.IsCellTransfer && t.CurrentScenarioSnapshot().Length == 0,
                "Scenario reset lost transfer actions or left its runtime mode installed.");
            var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position; item.Body.rotation = l.InputStop.rotation;
            Require(t.TryRegister(item, "NEW", ProductDestination.B, out e) && t.TryFeedNext(out _, out e) && c.TryPrepareScaraTransfer(out e), e);
            c.enabled = false; c.enabled = true; string before = Data(l);
            Require(c.TryCancelProgram(q, q.ProgramRevision, "준비만 한 7단계 경로 종료", out e), e);
            var cancelled = q.CancellationSnapshot().Last();
            Require(cancelled.IsCellTransfer && cancelled.Targets.Count == 7 && cancelled.ProductId == "NEW" && cancelled.GripCount == 0 &&
                cancelled.ReleaseCount == 0 && q.State == State.Cancelled && !c.HasControlledSequence && c.IsFaulted && t.InputProductId == "NEW" && Data(l) == before,
                "Cancellation lost the seven-step evidence or changed the prepared product.");
            Require(c.TryExecute(CellRunCommand.Reset, out e), e);
            Require(q.TryStart(out e), e);
            Require(!q.IsCellTransfer && !q.IsPickPlace && q.StepCount == 4 && q.CancellationSnapshot().Last() == cancelled && q.ScenarioResetSnapshot().Last() == archived,
                "Explicit legacy start retained the transfer mode or erased immutable history.");
            await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && q.StepIndex == 4 && q.GripCount == 0 && q.ReleaseCount == 0 && t.InputProductId == "NEW" && item.Owner == null,
                q.LastError ?? c.FaultReason ?? "Legacy program failed after a transfer cancellation/reset.");
            Require(t.TryGet("NEW", out var finalProduct) && finalProduct.Destination == ProductDestination.B,
                "Cancellation/legacy transition changed the new product destination.");
            Check(hashes); var report = new { passed = true, completedTransferPrograms = 1, completedLegacyPrograms = 1, archiveTargetCount = archived.Targets.Count,
                cancellationTargetCount = cancelled.Targets.Count, transferActionsPreserved = true, idleAndLegacyStepCount = 4, productHistory = t.Count,
                activeProduct = "NEW", destination = finalProduct.Destination.ToString(), sourcesPreserved = hashes.Count, directClicks = 0, callbacksOrUiInputInjected = 0, hashes };
            Write("scara-transfer-lifecycle.json", report); return JsonConvert.SerializeObject(new { report.passed, report.completedTransferPrograms, report.completedLegacyPrograms, report.archiveTargetCount, report.cancellationTargetCount });
        }
        finally { c.enabled = true; Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
    }

    public static async Task<string> SourceValidation(bool expectRejected)
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var q = l.Scara.GetComponent<RobotWaypointSequence>(); var original = q.Waypoints; var copy = UnityEngine.Object.Instantiate(original);
        var hashes = Sources();
        try
        {
            await Until(() => t.Count == 1); SetRetreat(copy, copy.Approach.PositionMetres);
            var so = new SerializedObject(q); so.FindProperty("waypoints").objectReferenceValue = copy; so.ApplyModifiedPropertiesWithoutUndo();
            Require(!copy.TryValidate(out _), "Expected an invalid existing waypoint source.");
            string before = Full(l); bool accepted = c.TryPrepareScaraTransfer(out var error);
            if (expectRejected) Require(!accepted && !string.IsNullOrEmpty(error) && Full(l) == before && !c.HasControlledSequence && q.State == State.Idle,
                "Invalid existing source partially installed an unstartable program.");
            else Require(accepted && q.State == State.Paused && !c.TryExecute(CellRunCommand.Start, out _), "Expected the invalid-source preparation gap.");
            Check(hashes); var report = new { passed = expectRejected, reproduced = !expectRejected && accepted, preparationAccepted = accepted,
                invalidSourceRejectedBeforeCommit = expectRejected, noMotion = l.Scara.CommandVersion == 0 && l.Fanuc.CommandVersion == 0,
                error, sourcesPreserved = hashes.Count, directClicks = 0, callbacksOrUiInputInjected = 0, hashes };
            Write("scara-transfer-invalid-source-" + (expectRejected ? "after" : "before") + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.reproduced, report.preparationAccepted, report.invalidSourceRejectedBeforeCommit, report.noMotion });
        }
        finally { var so = new SerializedObject(q); so.FindProperty("waypoints").objectReferenceValue = original; so.ApplyModifiedPropertiesWithoutUndo(); c.TryExecute(CellRunCommand.Stop, out _); UnityEngine.Object.Destroy(copy); }
    }

    public static async Task<string> RepeatTen()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = l.Scara.GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>();
        var hashes = Sources(); float scale = Time.timeScale; var ids = new List<string> { "P-000001" };
        var rows = new List<object>(); int grips = 0, releases = 0, queries = 0, rejects = 0, uiSnapshots = 0;
        string fanucBefore = FanucState(l); var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show();
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1);
            for (int i = 1; i < 10; i++)
            {
                string id = "SCARA-" + (i + 1).ToString("00"); ids.Add(id);
                var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * (i * .15f);
                item.Body.rotation = l.InputStop.rotation;
                Require(t.TryRegister(item, id, i % 2 == 0 ? ProductDestination.A : ProductDestination.B, out var e), e);
            }
            for (int i = 0; i < 10; i++)
            {
                string id = ids[i]; Require(t.TryGetItem(id, out var item), "Original product missing.");
                if (i > 0)
                {
                    item.Body.position = l.InputStop.position; item.Body.rotation = l.InputStop.rotation; await Ticks(2);
                    Require(t.TryFeedNext(out var fed, out var e) && fed.Id == id, e ?? "FIFO feed order changed.");
                }
                Require(t.TryGet(id, out var expected), "Input record missing.");
                Require(expected.Destination == (i % 2 == 0 ? ProductDestination.A : ProductDestination.B) &&
                    t.WaitingSnapshot().SequenceEqual(ids.Skip(i + 1)), "Input destination/FIFO changed.");
                string before = Full(l); for (int k = 0; k < 25; k++) { Require(c.CanPrepareScaraTransfer(out var e), e); queries++; }
                Require(Full(l) == before, "Read-only preparation query changed state.");
                ReadPrepareUi(w, true, id); uiSnapshots++;
                string data = Products(l), physical = Physical(l), motion = Motion(l);
                Require(c.TryPrepareScaraTransfer(expected, out var error), error);
                Require(c.State == CellRunState.Stopped && q.State == State.Paused && q.GripCount == 0 && q.ReleaseCount == 0 &&
                    Products(l) == data && Physical(l) == physical && Motion(l) == motion, "Preparation moved or changed products.");
                ReadPrepareUi(w, false, id); uiSnapshots++;
                before = Full(l); Require(!c.TryPrepareScaraTransfer(expected, out _) && Full(l) == before, "Duplicate preparation overwrote a path."); rejects++;
                Require(c.TryExecute(CellRunCommand.Start, out error), error);
                ReadPrepareUi(w, false, id); uiSnapshots++;
                before = Full(l); Require(!c.TryExecute(CellRunCommand.Start, out _) && Full(l) == before, "Duplicate start changed evidence."); rejects++;
                await Until(() =>
                {
                    if (item.Owner == g) Require(g.HeldItem == item && t.ScaraProductId == id && t.InputProductId == null,
                        "Physical owner and logical holding diverged.");
                    return q.State == State.Completed || c.IsFaulted;
                });
                Require(q.State == State.Completed && q.StepIndex == 7 && q.LastArrivedStepIndex == 6 && q.GripCount == 1 && q.ReleaseCount == 1 &&
                    item.Owner == null && g.HeldItem == null && t.HandoffProductId == id && t.InputProductId == null &&
                    t.ScaraProductId == null && t.Count == 10 && !c.IsFaulted && !t.IsFaulted && FanucState(l) == fanucBefore,
                    q.LastError ?? c.FaultReason ?? "Normal transfer failed.");
                Require(t.TryGet(id, out var placed) && placed.Location == ProductLocation.Handoff && placed.Destination == expected.Destination &&
                    t.WaitingSnapshot().SequenceEqual(ids.Skip(i + 1)), "Placement identity/destination/FIFO changed.");
                float positionError = Vector3.Distance(item.Body.position, l.HandoffPoint.position);
                float rotationError = RobotGripper.RotationError(item.Body.rotation, l.HandoffPoint.rotation);
                Require(positionError <= .002f && rotationError <= .5f, "Placement outside tolerance.");
                Require(l.Scara.TryReadFkDiagnostics(out var fk, out error), error);
                Require(Vector3.Distance(fk.ActualPositionMetres, l.ScaraHandoffWaypoints.Retreat.PositionMetres) <= .002f, "SCARA did not retreat.");
                grips += q.GripCount; releases += q.ReleaseCount;
                Require(c.TryExecute(CellRunCommand.Stop, out error) && c.TryReleaseControl(out error), error);
                ReadPrepareUi(w, false, null); uiSnapshots++;
                var snapshot = t.Snapshot(); int removed = snapshot.Count(r => r.Location == ProductLocation.Removed);
                Require(removed == i && snapshot.All(r => r.Location == ProductLocation.Removed || t.TryGetItem(r.Id, out var original) && original.IsReady && original.Owner == null) &&
                    snapshot.Count(r => r.Location == ProductLocation.OutputA || r.Location == ProductLocation.OutputB || r.Location == ProductLocation.Lost) == 0,
                    "Ten-product conservation or terminal classification failed.");
                rows.Add(new { cycle = i + 1, id, destination = placed.Destination.ToString(), grips = q.GripCount, releases = q.ReleaseCount,
                    positionErrorMetres = positionError, rotationErrorDegrees = rotationError, registered = 10, explicitRemoved = removed,
                    physicallyConfirmedResidual = 10 - removed, unconfirmed = 0 });
                // Test fixture cleanup only. This is not FANUC output or automatic line completion.
                if (i < 9) Require(t.TryRemove(id, ProductLocation.Handoff, out error), error);
            }
            Require(grips == 10 && releases == 10 && t.WaitingSnapshot().Count == 0 && t.HandoffProductId == ids[9] &&
                t.Snapshot().Count(r => r.Location == ProductLocation.Removed) == 9, "Final 10=9 removed+1 residual accounting failed.");
            Check(hashes); var report = new { passed = true, normalCycles = 10, automaticGrips = grips, automaticReleases = releases,
                destinationsA = 5, destinationsB = 5, registeredProducts = 10, completedProducts = 0, explicitRemoved = 9, physicallyConfirmedResidual = 1,
                unconfirmedProducts = 0, preparationQueriesPreserved = queries, rejectedRequestsPreserved = rejects, uiStateSnapshots = uiSnapshots,
                automaticFeedOrFanucCompletionImplemented = false, fixtureAlignedOriginalWaitingBodiesAndExplicitlyRemovedHandoff = true,
                fanucJointStatePreserved = FanucState(l) == fanucBefore, directClicks = 0, callbacksOrUiInputInjected = 0, rows, sourcesPreserved = hashes.Count, hashes };
            Write("scara-transfer-ten.json", report); return JsonConvert.SerializeObject(new { report.passed, report.normalCycles, report.automaticGrips, report.automaticReleases, report.explicitRemoved, report.physicallyConfirmedResidual, report.uiStateSnapshots });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
    }

    public static async Task<string> StalePrepare(int variant)
    {
        Require(variant == 0 || variant == 1, "Use replacement ID or same-ID return/feed.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); var hashes = Sources();
        await Until(() => t.Count == 1); Require(t.TryGet("P-000001", out var old), "Original record missing.");
        string before = Full(l); Require(!c.TryPrepareScaraTransfer(null, out _) && Full(l) == before, "Null preview prepared a path.");
        if (variant == 0)
        {
            Require(t.TryRemove(old.Id, ProductLocation.Input, out var e), e);
            var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position; item.Body.rotation = l.InputStop.rotation;
            Require(t.TryRegister(item, "REPLACEMENT", ProductDestination.B, out e) && t.TryFeedNext(out _, out e), e);
        }
        else
        {
            t.enabled = false; t.enabled = true;
            Require(t.TryReturnToQueue(old, "같은 ID 재투입 검사", out var e) && c.TryExecute(CellRunCommand.Reset, out e) && t.TryFeedNext(out _, out e), e);
        }
        Require(t.TryGet(t.InputProductId, out var current) && !ReferenceEquals(old, current), "Fixture did not replace the input record.");
        before = Full(l); Require(!c.TryPrepareScaraTransfer(old, out var error) && !string.IsNullOrEmpty(error) && Full(l) == before && !c.HasControlledSequence,
            "Stale displayed input prepared a different/current product.");
        var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show(); ReadPrepareUi(w, true, current.Id);
        string physical = Physical(l), motion = Motion(l);
        Require(c.TryPrepareScaraTransfer(current, out error) && Physical(l) == physical && Motion(l) == motion, error);
        ReadPrepareUi(w, false, current.Id); Check(hashes);
        var report = new { passed = true, variant, stalePreviewRejected = true, sameIdAndLocationDifferentRecord = variant == 1,
            nullPreviewRejected = true, freshPreviewPreparedWithoutMotion = true, directClicks = 0, callbacksOrUiInputInjected = 0,
            sourcesPreserved = hashes.Count, hashes };
        Write("scara-transfer-stale-" + variant + ".json", report); return JsonConvert.SerializeObject(new { report.passed, variant, report.stalePreviewRejected });
    }

    public static async Task<string> UiEdit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode."); var hashes = Sources();
        var scene = UnityEngine.SceneManagement.SceneManager.GetActiveScene(); string signature = scene.path + "/" + scene.isDirty;
        var c = UnityEngine.Object.FindFirstObjectByType<CellRunController>(); var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show();
        Rect original = w.position; int bounds = 0;
        try
        {
            ReadPrepareUi(w, false, null);
            foreach (float width in new[] { 440f, 520f })
            {
                w.position = new Rect(100, 80, width, 680); await Task.Delay(150);
                var root = w.rootVisualElement.worldBound;
                var advanced = w.rootVisualElement.Q<Foldout>("cell-run-individual-tests");
                if (advanced != null && !advanced.value) continue; // Individual preparation is intentionally folded.
                var r = w.rootVisualElement.Q<Button>("cell-run-prepare-scara-transfer").worldBound;
                Require(r.width >= 100 && r.height >= 25 && !float.IsNaN(r.x) && r.xMin >= root.xMin - 1 && r.xMax <= root.xMax + 1,
                    "Preparation button clips horizontally."); bounds++;
            }
            var editor = UnityEditor.Editor.CreateEditor(c);
            try { var root = editor.CreateInspectorGUI(); Require(!root.Q<Button>("cell-run-prepare-scara-transfer").enabledInHierarchy && root.Q<Label>("cell-run-transfer-target").text.Contains("Play 대기"), "Inspector edit preparation is enabled."); }
            finally { UnityEngine.Object.DestroyImmediate(editor); }
            Require(signature == scene.path + "/" + scene.isDirty, "UI reading changed scene/dirty state."); Check(hashes);
            var report = new { passed = true, editForms = 2, buttonBoundsChecks = bounds, widths = new[] { 440, 520 },
                directClicks = 0, callbacksOrUiInputInjected = 0, sourcesPreserved = hashes.Count, hashes };
            Write("scara-transfer-ui-edit.json", report); return JsonConvert.SerializeObject(new { report.passed, report.editForms, report.buttonBoundsChecks });
        }
        finally { w.position = original; }
    }

    public static async Task<string> UiPlay()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var q = l.Scara.GetComponent<RobotWaypointSequence>(); var hashes = Sources(); float scale = Time.timeScale; int views = 0;
        var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show();
        var editor = UnityEditor.Editor.CreateEditor(c);
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); Snap(true, "P-000001");
            Require(c.TryExecute(CellRunCommand.Start, out var e), e); Snap(false, "P-000001");
            Require(q.State == State.Idle, "Permission-only Start created a transfer without preparation.");
            Require(c.TryExecute(CellRunCommand.Pause, out e), e); Snap(false, "P-000001");
            Require(c.TryExecute(CellRunCommand.Stop, out e), e); Snap(true, "P-000001");
            Require(t.TryGet("P-000001", out var record) && c.TryPrepareScaraTransfer(record, out e), e); Snap(false, "P-000001");
            Require(w.rootVisualElement.Q<Button>("cell-run-start").enabledInHierarchy && c.State == CellRunState.Stopped && q.State == State.Paused,
                "Prepared UI auto-started or omitted the separate Start action.");
            Require(c.TryExecute(CellRunCommand.Start, out e), e); Snap(false, "P-000001");
            await Until(() => q.GripCount == 1 || c.IsFaulted); Require(!c.IsFaulted, q.LastError ?? c.FaultReason);
            Require(c.TryExecute(CellRunCommand.Pause, out e), e); Snap(false, null);
            Require(w.rootVisualElement.Q<Button>("cell-run-resume").enabledInHierarchy && !w.rootVisualElement.Q<Button>("cell-run-start").enabledInHierarchy,
                "Held interruption UI did not distinguish Resume from Start.");
            Require(c.TryExecute(CellRunCommand.Resume, out e), e); await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1, q.LastError ?? c.FaultReason); Snap(false, null);
            Require(c.TryExecute(CellRunCommand.Stop, out e) && c.TryReleaseControl(out e), e); Snap(false, null);
            Require(t.TryRemove("P-000001", ProductLocation.Handoff, out e), e); Snap(false, null);
            c.enabled = false; Snap(false, null); c.enabled = true; Snap(false, null);
            Require(c.IsFaulted && c.TryExecute(CellRunCommand.Reset, out e), e); Snap(false, null);
            UnityEngine.Object.DestroyImmediate(editor); editor = null;
            var rootBeforeDeletion = w.rootVisualElement;
            UnityEngine.Object.Destroy(c); await Until(() => c == null); await Task.Delay(150);
            Require(!rootBeforeDeletion.Q<Button>("cell-run-prepare-scara-transfer").enabledInHierarchy &&
                rootBeforeDeletion.Q<Label>("cell-run-transfer-target").text.Contains("확인 불가") && !q.TryStart(out _),
                "Deleted manager left preparation enabled or permitted a legacy fallback."); views++;
            Check(hashes); var report = new { passed = true, uiViewSnapshots = views, windowAndInspector = true, heldPauseThenResume = true,
                managerReactivationDoesNotRecover = true, managerDeletionLocksPreparationAndLegacy = true, actualGrips = q.GripCount, actualReleases = q.ReleaseCount,
                directClicks = 0, callbacksOrUiInputInjected = 0, actualButtonCallbackManualCheckPending = true, sourcesPreserved = hashes.Count, hashes };
            Write("scara-transfer-ui-play.json", report); return JsonConvert.SerializeObject(new { report.passed, report.uiViewSnapshots, report.managerDeletionLocksPreparationAndLegacy });
        }
        finally { Time.timeScale = scale; if (editor != null) UnityEngine.Object.DestroyImmediate(editor); if (c != null) { c.enabled = true; c.TryExecute(CellRunCommand.Stop, out _); } }
        void Snap(bool allowed, string id)
        {
            ReadPrepareUi(w, allowed, id);
            var root = editor.CreateInspectorGUI();
            Require(root.Q<Button>("cell-run-prepare-scara-transfer").enabledInHierarchy == allowed, "Inspector preparation permission differs.");
            if (id != null) Require(root.Q<Label>("cell-run-transfer-target").text.Contains(id), "Inspector preview omits input ID.");
            views += 2;
        }
    }

    private static void ReadPrepareUi(CellRunWindow w, bool allowed, string id)
    {
        w.RefreshCell(); var root = w.rootVisualElement;
        Require(root.Q<Button>("cell-run-prepare-scara-transfer").enabledInHierarchy == allowed, "Preparation button permission differs from API.");
        if (id != null)
        {
            var tracker = UnityEngine.Object.FindFirstObjectByType<CellProductTracker>();
            Require(tracker.TryGet(id, out var record) && root.Q<Label>("cell-run-transfer-target").text.Contains(id) &&
                root.Q<Label>("cell-run-transfer-target").text.Contains("목적지 " + record.Destination), "Preview omits current input identity/destination.");
        }
    }

    private static async Task Move(RobotGripper g, GrippableItem item, Transform station)
    {
        var point = item.GetGripPoint(RobotWaypointSet.RobotType.Scara);
        var rotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = station.TransformPoint(point.localPosition) - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        var local = Quaternion.Inverse(g.RobotFrame.rotation) * rotation; float yaw = Vector3.SignedAngle(Vector3.forward, local * Vector3.forward, Vector3.up);
        Require(g.Scara.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), yaw, ScaraJointLab.IkSelection.PositiveElbow, out _, out var e), e);
        await Until(() => g.IsStopped); await Ticks(2);
    }
    private static void SetDrop(LineWatchCellLayout l, RobotWaypointSet set)
    { var so = new SerializedObject(l); so.FindProperty("scaraHandoffWaypoints").objectReferenceValue = set; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetRetreat(RobotWaypointSet set, Vector3 p)
    { var so = new SerializedObject(set); so.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = p; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void Add(LineWatchCellLayout l, string id, float offset)
    { var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset; Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e); }
    private static LineWatchCellLayout Layout()
    { Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh Play."); var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>(); Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch."); return l; }
    private static string Products(LineWatchCellLayout l)
    { var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(), slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId } }); }
    private static string Physical(LineWatchCellLayout l) => JsonConvert.SerializeObject(UnityEngine.Object.FindObjectsByType<GrippableItem>(FindObjectsInactive.Include, FindObjectsSortMode.InstanceID).Select(i => new { id = Ref(i), i.ProductId, owner = Ref(i.Owner), p = V(i.Body.position), r = Q(i.Body.rotation), active = i.gameObject.activeInHierarchy }));
    private static string Motion(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { s = l.Scara.CommandVersion, f = l.Fanuc.CommandVersion,
        positions = new[] { l.Scara.Axis1.CurrentPosition, l.Scara.Axis2.CurrentPosition, l.Scara.Axis3.CurrentPosition, l.Scara.Axis4.CurrentPosition }.Concat(l.Fanuc.Joints.Select(j => j.CurrentPosition)),
        targets = new[] { l.Scara.Axis1.TargetPosition, l.Scara.Axis2.TargetPosition, l.Scara.Axis3.TargetPosition, l.Scara.Axis4.TargetPosition }.Concat(l.Fanuc.Joints.Select(j => j.TargetPosition)),
        held = new[] { Ref(l.Scara.GetComponent<RobotGripper>().HeldItem), Ref(l.Fanuc.GetComponent<RobotGripper>().HeldItem) } });
    private static string FanucState(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { current = l.Fanuc.Joints.Select(j => j.CurrentPosition), target = l.Fanuc.Joints.Select(j => j.TargetPosition), moving = l.Fanuc.IsTcpMoving, jointsMoving = l.Fanuc.Joints.Select(j => j.IsMoving) });
    private static string Data(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { products = Products(l), physical = Physical(l), motion = Motion(l),
        paths = new[] { l.Scara.GetComponent<RobotWaypointSequence>(), l.Fanuc.GetComponent<RobotWaypointSequence>() }.Select(q => new { q.StepIndex, q.IsPickPlace, q.IsCellTransfer, q.ProgramRevision, q.GripCount, q.ReleaseCount, q.LastArrivedStepIndex, last = V(q.LastArrivalPose.position) }) });
    private static string Full(LineWatchCellLayout l)
    { var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { data = Data(l), c.State, c.FaultReason, c.LastAcknowledgedFaultReason, c.HasControlledSequence, controlled = Ref(c.ControlledSequence), t.IsFaulted, tracker = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason, paths = new[] { l.Scara.GetComponent<RobotWaypointSequence>(), l.Fanuc.GetComponent<RobotWaypointSequence>() }.Select(q => new { q.State, q.LastError, q.LastAcknowledgedFaultReason }) }); }
    private static int Ref(UnityEngine.Object o) => ReferenceEquals(o, null) ? 0 : o.GetInstanceID();
    private static float[] V(Vector3 v) => new[] { v.x, v.y, v.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }.SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", ""); }
    private static void Check(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification modified source/assets.");
    private static async Task Ticks(int count) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= count * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 25, "Simulation wait timed out."); await Task.Delay(1); } }
    private static void Write(string file, object report) { Directory.CreateDirectory("Logs/LW12"); File.WriteAllText("Logs/LW12/" + file, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
