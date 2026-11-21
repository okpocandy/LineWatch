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

public static class VerifyFanucATransfer
{
    public static Task<string> Normal(int interruption) => RunNormal(interruption, ProductDestination.A);
    public static Task<string> NormalB(int interruption) => RunNormal(interruption, ProductDestination.B);

    private static async Task<string> RunNormal(int interruption, ProductDestination destination)
    {
        Require(interruption >= 0 && interruption <= 5, "Use normal/Pause/Stop/path fault/post-release Pause 0..5 (5: frozen output change).");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = l.Fanuc.GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>();
        var outputStation = destination == ProductDestination.A ? l.OutputA : l.OutputB; var outputPosition = outputStation.position;
        int removed = destination == ProductDestination.B ? 1 : 0;
        var source = l.FanucHandoffWaypoints; var copy = UnityEngine.Object.Instantiate(source); var hashes = Sources(); float scale = Time.timeScale;
        int rejects = 0;
        try
        {
            Time.timeScale = 5; var item = await SupplyHandoff(l, destination, true); SetPick(l, copy);
            string scara = ScaraState(l), products = Products(l), physical = Physical(l), motion = Motion(l), before = Full(l);
            for (int i = 0; i < 100; i++) Require(CanPrepare(c, destination, out var e), e);
            Require(Full(l) == before, "FANUC preparation query changed evidence.");
            Require(t.TryGet(item.ProductId, out var expected), "Handoff record missing.");
            if (interruption == 0)
            {
                var opposite = destination == ProductDestination.A ? ProductDestination.B : ProductDestination.A;
                Reject(() => c.TryPrepareFanucTransfer(opposite, out _), "wrong destination");
                Reject(() => c.TryPrepareFanucTransfer(opposite, expected, out _), "wrong destination preview");
                Reject(() => c.TryPrepareFanucTransfer((ProductDestination)99, out _), "invalid destination");
                Reject(() => c.CanPrepareFanucTransfer((ProductDestination)99, out _), "invalid destination query");
                if (destination == ProductDestination.B)
                {
                    Reject(() => c.TryPrepareFanucTransferToA(out _), "A compatibility API with B item");
                    Reject(() => c.TryPrepareFanucTransferToA(expected, out _), "A compatibility preview with B item");
                }
            }
            Require(Prepare(c, destination, expected, out var error), error);
            Require(c.State == CellRunState.Stopped && c.ControlledSequence == q && q.State == State.Paused && q.IsFanucTransfer && q.FanucDestination == destination && q.StepCount == 7 &&
                q.GripCount == 0 && q.ReleaseCount == 0 && Products(l) == products && Physical(l) == physical && Motion(l) == motion,
                "Preparation changed products, dispatched a command or failed to adopt the seven-step FANUC path.");
            var inspector = UnityEditor.Editor.CreateEditor(q); var panel = inspector.CreateInspectorGUI();
            Require(panel.Query<Label>().ToList().Any(label => label.text.Contains(destination + " 출구 접근")), "FANUC route label has the wrong destination.");
            UnityEngine.Object.DestroyImmediate(inspector);
            before = Full(l); await Ticks(3); Require(Full(l) == before, "Preparation auto-started.");
            Reject(() => Prepare(c, destination, expected, out _), "duplicate preparation");
            Reject(() => l.Scara.GetComponent<RobotWaypointSequence>().TryStart(out _), "other robot start");
            Reject(() => g.TryGrip(item, out _), "manual grip");
            Require(c.TryExecute(CellRunCommand.Start, out error), error);
            Reject(() => c.TryExecute(CellRunCommand.Start, out _), "duplicate start");
            if (interruption != 0)
            {
                await Until(() => interruption == 4 ? q.ReleaseCount == 1 || c.IsFaulted : q.GripCount == 1 || c.IsFaulted);
                Require(!c.IsFaulted, q.LastError ?? c.FaultReason);
                if (interruption == 3 || interruption == 5)
                {
                    var retreat = copy.Retreat.PositionMetres;
                    if (interruption == 3) SetRetreat(copy, retreat + Vector3.right * .01f);
                    else outputStation.position += Vector3.right * .01f;
                    await Until(() => q.State == State.Faulted && c.IsFaulted); SetRetreat(copy, retreat); outputStation.position = outputPosition;
                }
                else Require(c.TryExecute(interruption == 2 ? CellRunCommand.Stop : CellRunCommand.Pause, out error), error);
                await Ticks(2); string data = Data(l); before = Full(l); await Ticks(3); Require(Full(l) == before, "Interruption changed holding, identity or step.");
                Reject(() => g.TryRelease(out _), "manual release"); Reject(() => q.TryResume(out _), "individual resume");
                var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show(); w.RefreshCell();
                Require(w.rootVisualElement.Q<Label>("cell-run-fanuc").text.Contains("/7"), "FANUC transfer UI reports the old step count.");
                if (interruption == 3 || interruption == 5)
                {
                    Require(c.TryExecute(CellRunCommand.Reset, out error), error);
                    Require(c.State == CellRunState.Stopped && q.State == State.Paused && Data(l) == data, "Fault acknowledgement changed actions/product evidence.");
                    before = Full(l); await Ticks(2); Require(Full(l) == before, "Acknowledgement automatically restarted.");
                }
                Require(c.TryExecute(c.State == CellRunState.Paused ? CellRunCommand.Resume : CellRunCommand.Start, out error), error);
            }
            await Until(() =>
            {
                if (item.Owner == g) Require(g.HeldItem == item && t.FanucProductId == item.ProductId && t.HandoffProductId == null,
                    "FANUC Owner and logical slot diverged.");
                return q.State == State.Completed || c.IsFaulted;
            });
            Require(q.State == State.Completed && q.StepIndex == 7 && q.LastArrivedStepIndex == 6 && q.GripCount == 1 && q.ReleaseCount == 1 &&
                item.Owner == null && g.HeldItem == null && t.HandoffProductId == null && t.FanucProductId == null && t.Count == 3 + removed &&
                t.TryGet(item.ProductId, out var result) && result.Location == (destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB) && result.Destination == destination &&
                t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B" }) && !t.IsFaulted && !c.IsFaulted && ScaraState(l) == scara,
                q.LastError ?? c.FaultReason ?? "FANUC A transfer failed or changed source ownership/FIFO/SCARA pose.");
            float positionError = Vector3.Distance(item.Body.position, outputStation.position);
            float rotationError = RobotGripper.RotationError(item.Body.rotation, outputStation.rotation);
            Require(positionError <= .002f && rotationError <= .5f && t.Snapshot().Count(r => r.Location == (destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB)) == 1 &&
                t.Snapshot().Count(r => r.Location == (destination == ProductDestination.A ? ProductLocation.OutputB : ProductLocation.OutputA) || r.Location == ProductLocation.Lost) == 0 &&
                t.Snapshot().Count(r => r.Location == ProductLocation.Removed) == removed,
                "Output pose or completed/explicitly removed/physical residual accounting failed.");
            Require(c.TryExecute(CellRunCommand.Stop, out error), error); products = Products(l); motion = Motion(l);
            Require(c.TryExecute(CellRunCommand.Start, out error), error); await Ticks(2);
            Require(Products(l) == products && Motion(l) == motion && q.GripCount == 1 && q.ReleaseCount == 1, "Completed FANUC path replayed.");
            Require(c.TryExecute(CellRunCommand.Stop, out error) && c.TryReleaseControl(out error), error);
            Reject(() => Prepare(c, destination, out _), "empty handoff after completion");
            Check(hashes); var report = new { passed = true, interruption, completedFanucPrograms = 1, automaticGrips = q.GripCount,
                automaticReleases = q.ReleaseCount, registeredProducts = 3 + removed, completedA = destination == ProductDestination.A ? 1 : 0, completedB = destination == ProductDestination.B ? 1 : 0, explicitRemoved = removed,
                physicallyConfirmedResidual = 2, unconfirmedProducts = 0, preparationQueriesPreserved = 100, rejectedRequestsPreserved = rejects,
                placementErrorMetres = positionError, placementRotationErrorDegrees = rotationError, scaraRetreatPreserved = true,
                fixtureUsedSeparateScaraPrepareStartStopAndRelease = true, automaticTwoRobotDispatchImplemented = false,
                directClicks = 0, callbacksOrUiInputInjected = 0, sourcesPreserved = hashes.Count, hashes };
            Write("fanuc-" + destination.ToString().ToLowerInvariant() + "-normal-" + interruption + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, interruption, report.automaticGrips, report.automaticReleases, report.placementErrorMetres, report.rejectedRequestsPreserved });
        }
        finally { outputStation.position = outputPosition; Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetPick(l, source); UnityEngine.Object.Destroy(copy); }
        void Reject(Func<bool> action, string name)
        { string before = Full(l); Require(!action() && Full(l) == before, "Rejected request changed evidence: " + name); rejects++; }
    }

    public static Task<string> Blocked(int scenario) => RunBlocked(scenario, ProductDestination.A);
    public static Task<string> BlockedB(int scenario) => RunBlocked(scenario, ProductDestination.B);

    private static async Task<string> RunBlocked(int scenario, ProductDestination destination)
    {
        Require(scenario >= 0 && scenario <= 6, "Use blocked 0..6.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = l.Fanuc.GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>(); var sg = l.Scara.GetComponent<RobotGripper>();
        var outputStation = destination == ProductDestination.A ? l.OutputA : l.OutputB;
        var hashes = Sources(); float scale = Time.timeScale; Vector3 output = outputStation.position, tcp = l.Scara.Tcp.localPosition; int rejects = 0;
        var scaraTemplate = l.ScaraHandoffWaypoints; RobotWaypointSet wrongScara = null;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1);
            if (scenario == 0) Reject("empty handoff");
            else
            {
                var item = await SupplyHandoff(l, scenario == 1 ? (destination == ProductDestination.A ? ProductDestination.B : ProductDestination.A) : destination, true);
                if (scenario == 1) Reject("B destination cannot go to A");
                else if (scenario == 2)
                {
                    var work = l.ScaraHandoffWaypoints.Work;
                    Require(l.Scara.TryMoveTcp(work.PositionMetres, Yaw(work.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out var e), e);
                    await Until(() => sg.IsStopped); Reject("SCARA not retreated");
                    var retreat = l.ScaraHandoffWaypoints.Retreat;
                    Require(l.Scara.TryMoveTcp(retreat.PositionMetres, Yaw(retreat.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out e), e);
                    await Until(() => sg.IsStopped);
                    string before = Full(l); await Ticks(2); Require(Full(l) == before && q.State == State.Idle, "Retreat repair auto-prepared FANUC.");
                    Require(CanPrepare(c, destination, out e), e);
                }
                else if (scenario == 3)
                {
                    var lab = sg.GetComponent<RobotGripperLab>();
                    Require(lab.TryPrepare(out var e) && lab.TryMoveToWork(out e), e); await Until(() => sg.IsStopped);
                    Require(sg.TryGrip(lab.TrialItem, out e), e); Reject("other actual holding");
                }
                else if (scenario == 4)
                {
                    outputStation.position += Vector3.right * 9f; Reject("unreachable A goal");
                    Require(q.State == State.Idle && !q.IsFanucTransfer && !c.HasControlledSequence && q.StepCount == 4,
                        "Invalid final target partially assigned the seven-step path.");
                    outputStation.position = output;
                    wrongScara = UnityEngine.Object.Instantiate(scaraTemplate);
                    var wrongData = new SerializedObject(wrongScara); wrongData.FindProperty("robot").enumValueIndex = 1; wrongData.ApplyModifiedPropertiesWithoutUndo();
                    SetScaraTemplate(l, wrongScara); Reject("wrong SCARA waypoint robot kind"); SetScaraTemplate(l, scaraTemplate);
                    Time.timeScale = 0; Reject("time stopped"); Time.timeScale = 5;
                    EditorApplication.isPaused = true; Reject("Editor paused"); EditorApplication.isPaused = false;
                    c.enabled = false; Reject("manager disabled"); c.enabled = true; Reject("reactivation still faulted");
                    Require(c.TryExecute(CellRunCommand.Reset, out var e), e);
                }
                else
                {
                    Require(Prepare(c, destination, out var e) && c.TryExecute(CellRunCommand.Start, out e), e);
                    var point = item.GetGripPoint(RobotWaypointSet.RobotType.Fanuc); var pointPosition = point.localPosition;
                    if (scenario == 5)
                    {
                        await Until(() => q.GripCount == 1 || c.IsFaulted); Require(!c.IsFaulted, q.LastError ?? c.FaultReason);
                        l.Scara.Tcp.localPosition += Vector3.right * .01f;
                    }
                    else point.localPosition += Vector3.up * .02f;
                    await Until(() => q.State == State.Faulted && c.IsFaulted); await Ticks(2);
                    Require(q.GripCount == (scenario == 5 ? 1 : 0) && q.ReleaseCount == 0 &&
                        (scenario == 5 ? item.Owner == g && g.HeldItem == item && t.FanucProductId == item.ProductId
                            : item.Owner == null && g.HeldItem == null && t.HandoffProductId == item.ProductId),
                        "Safety/grip failure changed physical ownership or repeated an action.");
                    string before = Full(l); Require(!c.TryExecute(CellRunCommand.Reset, out _) && Full(l) == before,
                        "Unresolved TCP/grip evidence was acknowledged."); rejects++;
                    l.Scara.Tcp.localPosition = tcp; point.localPosition = pointPosition; await Ticks(2);
                    Require(c.IsFaulted && q.State == State.Faulted, "Physical repair implicitly cleared faults.");
                    Require(c.TryExecute(CellRunCommand.Reset, out e) && c.TryExecute(CellRunCommand.Start, out e), e);
                    await Until(() => q.State == State.Completed || c.IsFaulted);
                    Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1 &&
                        t.TryGet(item.ProductId, out var result) && result.Location == (destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB),
                        q.LastError ?? c.FaultReason ?? "Separate recovery failed or repeated a grip/release.");
                }
            }
            Check(hashes); var report = new { passed = true, scenario, rejectedRequestsPreserved = rejects,
                explicitRecoveryCompleted = scenario >= 5, sourcesPreserved = hashes.Count, directClicks = 0, callbacksOrUiInputInjected = 0, hashes };
            Write("fanuc-" + destination.ToString().ToLowerInvariant() + "-blocked-" + scenario + ".json", report); return JsonConvert.SerializeObject(new { report.passed, scenario, report.rejectedRequestsPreserved, report.explicitRecoveryCompleted });
        }
        finally { EditorApplication.isPaused = false; c.enabled = true; outputStation.position = output; l.Scara.Tcp.localPosition = tcp; SetScaraTemplate(l, scaraTemplate); if (wrongScara != null) UnityEngine.Object.Destroy(wrongScara); Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
        void Reject(string name) { string before = Full(l); Require(!Prepare(c, destination, out var e) && !string.IsNullOrEmpty(e) && Full(l) == before, "Rejected FANUC preparation mutated state: " + name); rejects++; }
    }

    public static Task<string> Stale() => RunStale(ProductDestination.A);
    public static Task<string> StaleB() => RunStale(ProductDestination.B);

    private static async Task<string> RunStale(ProductDestination destination)
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); var hashes = Sources(); float scale = Time.timeScale;
        try
        {
            Time.timeScale = 5; var item = await SupplyHandoff(l, destination, false);
            Require(t.TryGet(item.ProductId, out var old), "Old handoff missing.");
            string before = Full(l); Require(!Prepare(c, destination, null, out _) && Full(l) == before, "Null preview prepared FANUC.");
            t.enabled = false; t.enabled = true;
            Require(t.TryReturnToQueue(old, "같은 인계 ID 재처리 검사", out var e) && c.TryExecute(CellRunCommand.Reset, out e), e);
            item.Body.position = l.InputStop.position; item.Body.rotation = l.InputStop.rotation; await Ticks(2);
            Require(t.TryFeedNext(out _, out e) && c.TryPrepareScaraTransfer(out e) && c.TryExecute(CellRunCommand.Start, out e), e);
            var sq = l.Scara.GetComponent<RobotWaypointSequence>(); await Until(() => sq.State == State.Completed || c.IsFaulted);
            Require(sq.State == State.Completed && c.TryExecute(CellRunCommand.Stop, out e) && c.TryReleaseControl(out e), e);
            Require(t.TryGet(item.ProductId, out var current) && current.Location == old.Location && !ReferenceEquals(current, old), "Same-ID/location handoff fixture failed.");
            before = Full(l); Require(!Prepare(c, destination, old, out _) && Full(l) == before && !c.HasControlledSequence,
                "Stale handoff record prepared a newer job.");
            string physical = Physical(l), motion = Motion(l);
            Require(Prepare(c, destination, current, out e) && Physical(l) == physical && Motion(l) == motion, e);
            Check(hashes); var report = new { passed = true, staleSameIdLocationRejected = true, nullPreviewRejected = true,
                currentRecordPreparedWithoutMotion = true, sourcesPreserved = hashes.Count, directClicks = 0, callbacksOrUiInputInjected = 0, hashes };
            Write("fanuc-" + destination.ToString().ToLowerInvariant() + "-stale.json", report); return JsonConvert.SerializeObject(new { report.passed, report.staleSameIdLocationRejected, report.currentRecordPreparedWithoutMotion });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
    }

    public static Task<string> Lifecycle() => RunLifecycle(ProductDestination.A);
    public static Task<string> LifecycleB() => RunLifecycle(ProductDestination.B);

    private static async Task<string> RunLifecycle(ProductDestination destination)
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var q = l.Fanuc.GetComponent<RobotWaypointSequence>(); var hashes = Sources(); float scale = Time.timeScale;
        try
        {
            Time.timeScale = 5; var item = await SupplyHandoff(l, destination, false);
            Require(Prepare(c, destination, out var e) && c.TryExecute(CellRunCommand.Start, out e), e);
            await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1, q.LastError ?? c.FaultReason);
            Require(c.TryExecute(CellRunCommand.Stop, out e) && c.TryReleaseControl(out e), e);
            var position = item.Body.position; var rotation = item.Body.rotation; string products = Products(l), motion = Motion(l);
            Require(c.TryResetScenario("FANUC 완료/행동 이력 보존", out e), e);
            var record = q.ScenarioResetSnapshot().Last();
            Require(record.IsFanucTransfer && record.FanucDestination == destination && record.IsCellTransfer && record.Targets.Count == 7 && record.GripCount == 1 && record.ReleaseCount == 1 &&
                record.ProductId == item.ProductId && q.State == State.Idle && q.StepCount == 4 && !q.IsFanucTransfer && q.FanucDestination == null &&
                Products(l) == products && Motion(l) == motion && item.Body.position == position && item.Body.rotation == rotation &&
                t.Count == (destination == ProductDestination.A ? 1 : 2) && t.CurrentScenarioSnapshot().Length == 0, "Reset lost FANUC targets/actions, completed product or physical pose.");
            Require(q.TryStart(out e), e);
            Require(!q.IsCellTransfer && q.StepCount == 4, "Legacy FANUC start retained the A-transfer mode.");
            await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && q.StepIndex == 4 && q.GripCount == 0 && q.ReleaseCount == 0 && q.ScenarioResetSnapshot().Last() == record &&
                t.TryGet(item.ProductId, out var product) && product.Location == (destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB) && product.Destination == destination &&
                item.Owner == null && item.Body.position == position && item.Body.rotation == rotation,
                q.LastError ?? c.FaultReason ?? "Legacy movement changed A completion or history.");
            Check(hashes); var report = new { passed = true, completedFanucTransfers = 1, legacyPrograms = 1, archivedTargets = 7,
                completedProductAndActualPosePreserved = true, cumulativeProductCount = t.Count, currentProductCount = t.CurrentScenarioSnapshot().Length,
                sourcesPreserved = hashes.Count, directClicks = 0, callbacksOrUiInputInjected = 0, hashes };
            Write("fanuc-" + destination.ToString().ToLowerInvariant() + "-lifecycle.json", report); return JsonConvert.SerializeObject(new { report.passed, report.completedFanucTransfers, report.legacyPrograms, report.archivedTargets });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
    }

    public static Task<string> ContinuousPair() => RunContinuousPair(false);
    public static Task<string> ContinuousUi() => RunContinuousPair(true);

    private static async Task<string> RunContinuousPair(bool verifyUi)
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sq = l.Scara.GetComponent<RobotWaypointSequence>(); var fq = l.Fanuc.GetComponent<RobotWaypointSequence>();
        var fg = l.Fanuc.GetComponent<RobotGripper>(); float scale = Time.timeScale; int uiViews = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1);
            string firstId = t.InputProductId; Require(t.TryGetItem(firstId, out var first), "First original missing.");
            var second = UnityEngine.Object.Instantiate(l.ProductPrefab); second.Body.position = l.InputStop.position + Vector3.right * .4f;
            second.Body.rotation = l.InputStop.rotation * Quaternion.Euler(0, 30, 0);
            Require(t.TryRegister(second, "CONT-B", ProductDestination.B, out var error), error);
            string products = Products(l), motion = Motion(l);
            if (verifyUi)
            {
                ReadUi(true); await Task.Delay(120);
                var rootBounds = EditorWindow.GetWindow<CellRunWindow>().rootVisualElement.worldBound;
                var bounds = EditorWindow.GetWindow<CellRunWindow>().rootVisualElement.Q<Button>("cell-run-prepare-continuous-transfer").worldBound;
                Require(bounds.width >= 100 && bounds.xMin >= rootBounds.xMin - 1 && bounds.xMax <= rootBounds.xMax + 1, "Continuous preparation clips horizontally.");
            }
            Require(t.TryGet(firstId, out var preview), "Initial preview missing.");
            Require(c.TryPrepareContinuousTransfer(preview, out error), error);
            Require(c.IsAutomaticContinuous && c.State == CellRunState.Stopped && Products(l) == products && Motion(l) == motion, "Continuous preparation changed products/motion.");
            if (verifyUi) ReadUi(false);
            Require(c.TryExecute(CellRunCommand.Start, out error), error);
            if (verifyUi) ReadUi(false);
            await Until(() => c.AutomaticPhase == CellRunController.AutomaticTransferPhase.WaitingForInput || c.IsFaulted);
            Require(t.TryGet("CONT-B", out var waiting), "Waiting record missing.");
            Require(!c.IsFaulted && c.State == CellRunState.Running && c.AutomaticProductId == "CONT-B" && c.AutomaticDestination == ProductDestination.B &&
                t.TryGet(firstId, out var firstResult) && firstResult.Location == ProductLocation.OutputA &&
                waiting.Location == ProductLocation.Waiting && t.InputProductId == null &&
                t.WaitingSnapshot().SequenceEqual(new[] { "CONT-B" }) && first.Owner == null && second.Owner == null,
                c.FaultReason ?? "Continuous alignment wait failed.");
            int scaraGrips = sq.GripCount, scaraReleases = sq.ReleaseCount, fanucGrips = fq.GripCount, fanucReleases = fq.ReleaseCount;
            Vector3 supplyStart = second.Body.position; double supplyTime = Time.fixedTimeAsDouble;
            await Until(() => Vector3.Distance(second.Body.position, supplyStart) > .03f || c.IsFaulted);
            float suppliedMetres = Vector3.Distance(second.Body.position, supplyStart);
            Require(!c.IsFaulted && c.AutomaticPhase == CellRunController.AutomaticTransferPhase.WaitingForInput &&
                Vector3.Distance(second.Body.position, l.InputStop.position) > .1f &&
                suppliedMetres <= .2f * (float)(Time.fixedTimeAsDouble - supplyTime) + .005f && t.InputProductId == null,
                "Supply teleported, exceeded the speed bound, or consumed FIFO before alignment.");
            var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
            string status = window.rootVisualElement.Q<Label>("cell-run-automatic-transfer").text;
            Require(status.Contains("연속 운전") && status.Contains("CONT-B") && status.Contains("투입 공급/정렬"), "Continuous supply display mismatch.");
            CheckCompactControls(window.rootVisualElement);
            if (verifyUi) ReadUi(false);
            else
            {
                Require(c.TryExecute(CellRunCommand.Pause, out error), error); string before = Full(l);
                Require(!t.TryFeedNext(out _, out _) && Full(l) == before, "Manual feed bypassed continuous ownership.");
                await Task.Delay(150); Require(Full(l) == before, "Paused supply changed body poses or FIFO evidence.");
                Require(c.State == CellRunState.Paused && t.InputProductId == null && t.WaitingSnapshot().SequenceEqual(new[] { "CONT-B" }) &&
                    t.TryGet("CONT-B", out var stillWaiting) && ReferenceEquals(waiting, stillWaiting), "Pause consumed the waiting product.");
                Require(c.TryExecute(CellRunCommand.Resume, out error), error);
            }
            await Until(() =>
            {
                if (second.Owner == fg) Require(fg.HeldItem == second && t.FanucProductId == "CONT-B" && t.HandoffProductId == null, "Second physical/logical holding diverged.");
                return c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Completed || c.IsFaulted;
            });
            Require(!c.IsFaulted && !t.IsFaulted && c.State == CellRunState.Stopped && c.IsAutomaticContinuous &&
                t.TryGet("CONT-B", out var secondResult) && secondResult.Location == ProductLocation.OutputB && secondResult.Destination == ProductDestination.B &&
                t.Count == 2 && t.WaitingSnapshot().Count == 0 && t.InputProductId == null && t.HandoffProductId == null && t.ScaraProductId == null && t.FanucProductId == null &&
                first.Owner == null && second.Owner == null && l.Scara.GetComponent<RobotGripper>().HeldItem == null && fg.HeldItem == null &&
                scaraGrips == 1 && scaraReleases == 1 && fanucGrips == 1 && fanucReleases == 1 && sq.GripCount == 1 && sq.ReleaseCount == 1 && fq.GripCount == 1 && fq.ReleaseCount == 1 &&
                Vector3.Distance(first.Body.position, l.OutputA.position) <= .002f && Vector3.Distance(second.Body.position, l.OutputB.position) <= .002f &&
                RobotGripper.RotationError(first.Body.rotation, l.OutputA.rotation) <= .5f && RobotGripper.RotationError(second.Body.rotation, l.OutputB.rotation) <= .5f,
                c.FaultReason ?? sq.LastError ?? fq.LastError ?? "Continuous A/B completion/accounting failed.");
            if (verifyUi) ReadUi(false);
            Require(c.TryReleaseControl(out error) && !c.IsAutomaticContinuous && c.AutomaticPhase == CellRunController.AutomaticTransferPhase.None, error);
            if (verifyUi) ReadUi(false);
            var report = new { passed = true, products = 2, uiViews, completedA = 1, completedB = 1, scaraGrips = scaraGrips + sq.GripCount,
                scaraReleases = scaraReleases + sq.ReleaseCount, fanucGrips = fanucGrips + fq.GripCount, fanucReleases = fanucReleases + fq.ReleaseCount,
                initialStarts = 1, explicitPauseResume = verifyUi ? 0 : 1, supplyMovedGradually = true, pauseRetested = !verifyUi, pausedSupplyAndFifoPreserved = verifyUi ? (bool?)null : true, observedSupplyMetres = suppliedMetres,
                unconfirmed = 0, manualFeedRejected = verifyUi ? (bool?)null : true, fixturePhysicallyAlignedSecondProduct = false, automaticPhysicalSupplyImplemented = true,
                directClicks = 0, callbacksOrUiInputInjected = 0 };
            Directory.CreateDirectory("Logs/LW12A"); File.WriteAllText(verifyUi ? "Logs/LW12A/continuous-ui.json" : "Logs/LW12A/supply-pair.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return JsonConvert.SerializeObject(report);
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
        void ReadUi(bool mayPrepare)
        {
            var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell(); Check(window.rootVisualElement);
            var inspector = UnityEditor.Editor.CreateEditor(c);
            try { Check(inspector.CreateInspectorGUI()); }
            finally { UnityEngine.Object.DestroyImmediate(inspector); }
            void Check(VisualElement root)
            {
                CheckCompactControls(root);
                Require(root.Q<Button>("cell-run-prepare-continuous-transfer").enabledInHierarchy == mayPrepare &&
                    root.Q<Button>("cell-run-prepare-automatic-transfer").enabledInHierarchy == mayPrepare, "Preparation mode lock mismatch.");
                if (c.AutomaticPhase != CellRunController.AutomaticTransferPhase.None)
                {
                    string status = root.Q<Label>("cell-run-automatic-transfer").text;
                    Require(status.Contains("연속 운전") && status.Contains(c.AutomaticProductId) && status.Contains("목적지 " + c.AutomaticDestination), "Continuous mode/identity display mismatch.");
                }
                if (c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Completed)
                    Require(!root.Q<Button>("cell-run-start").enabledInHierarchy, "Completed cycle exposes replay.");
                uiViews++;
            }
        }
    }

    public static object CompactWindow()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode for window cleanup.");
        var old = Resources.FindObjectsOfTypeAll<EditorWindow>().Where(w => w is ScaraJointLabWindow || w is FanucJointLabWindow).ToArray();
        foreach (var w in old) w.Close();
        var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
        CheckCompactControls(window.rootVisualElement);
        Require(!window.rootVisualElement.Q<Button>("cell-run-prepare-automatic-transfer").enabledInHierarchy, "Edit preparation is enabled.");
        var report = new { passed = true, closedIndividualLabWindows = old.Length, individualAndRecoveryFolded = true, directClicks = 0, callbacksOrUiInputInjected = 0 };
        Directory.CreateDirectory("Logs/LW12A"); File.WriteAllText("Logs/LW12A/window-compact.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;
    }

    private static void CheckCompactControls(VisualElement root)
    {
        var individual = root.Q<Foldout>("cell-run-individual-tests"); var recovery = root.Q<Foldout>("cell-run-recovery-tools");
        Require(individual != null && !individual.value && recovery != null && !recovery.value, "Advanced tools are not folded by default.");
        Require(individual.Contains(root.Q<Button>("cell-run-prepare-scara-transfer")) && individual.Contains(root.Q<Button>("cell-run-prepare-fanuc-transfer")) &&
            individual.Contains(root.Q<Button>("cell-run-take-scara")) && individual.Contains(root.Q<Button>("cell-run-take-fanuc")), "Individual tests leaked into the main controls.");
        Require(root.Q<Button>("cell-run-prepare-automatic-transfer").parent.parent.name == "cell-run-controls" &&
            root.Q<Button>("cell-run-prepare-continuous-transfer").parent.parent.name == "cell-run-controls", "Primary preparation modes are hidden.");
    }

    public static Task<string> AutomaticUi() => AutomaticSingle(3);

    public static async Task<string> AutomaticSingle(int scenario)
    {
        Require(scenario >= 0 && scenario <= 3, "Use A/B normal, pause/handoff-fault recovery, or automatic UI.");
        var destination = scenario == 1 ? ProductDestination.B : ProductDestination.A;
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sq = l.Scara.GetComponent<RobotWaypointSequence>(); var fq = l.Fanuc.GetComponent<RobotWaypointSequence>();
        var fg = l.Fanuc.GetComponent<RobotGripper>(); float scale = Time.timeScale; Vector3 outputPosition = l.OutputA.position;
        bool handoffObserved = false;
        int uiViews = 0; var window = scenario == 3 ? EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작") : null;
        if (window != null) window.Show();
        try
        {
            Time.timeScale = scenario == 2 ? 2 : 5; await Until(() => t.Count == 1);
            if (destination == ProductDestination.B)
            {
                Require(t.TryRemove(t.InputProductId, ProductLocation.Input, out var e), e);
                var b = UnityEngine.Object.Instantiate(l.ProductPrefab); b.Body.position = l.InputStop.position; b.Body.rotation = l.InputStop.rotation;
                Require(t.TryRegister(b, "AUTO-B", destination, out e) && t.TryFeedNext(out _, out e), e);
            }
            string id = t.InputProductId; Require(t.TryGetItem(id, out var item), "Input item missing.");
            Require(t.TryGet(id, out var expected), "Input record missing.");
            Add(l, "AUTO-WAIT", .4f);
            if (window != null)
            {
                ReadAutomaticUi(true, id, CellRunController.AutomaticTransferPhase.None); await Task.Delay(120);
                var bounds = window.rootVisualElement.worldBound;
                var button = window.rootVisualElement.Q<Button>("cell-run-prepare-automatic-transfer").worldBound;
                Require(button.width >= 100 && button.xMin >= bounds.xMin - 1 && button.xMax <= bounds.xMax + 1, "Automatic preparation button clips horizontally.");
            }
            string products = Products(l), motion = Motion(l), fanucBefore = FanucPosition();
            Require(c.TryPrepareAutomaticTransfer(expected, out var error), error);
            Require(c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Scara && c.AutomaticProductId == id && c.AutomaticDestination == destination &&
                c.State == CellRunState.Stopped && sq.State == State.Paused && c.ControlledSequence == sq && Products(l) == products && Motion(l) == motion,
                "Automatic preparation changed products/motion or failed to adopt SCARA.");
            if (window != null) ReadAutomaticUi(false, id, CellRunController.AutomaticTransferPhase.Scara);
            string before = Full(l);
            Require(!c.TryPrepareAutomaticTransfer(out _) && !fq.TryStart(out _) && Full(l) == before, "Duplicate/other-robot start changed preparation.");
            Require(c.TryExecute(CellRunCommand.Start, out error), error);
            if (window != null) ReadAutomaticUi(false, id, CellRunController.AutomaticTransferPhase.Scara);
            before = Full(l); Require(!c.TryExecute(CellRunCommand.Start, out _) && Full(l) == before, "Duplicate common start changed the cycle.");
            if (scenario == 2)
            {
                await Until(() => sq.ReleaseCount == 1 || c.IsFaulted);
                Require(!c.IsFaulted && c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Scara && FanucPosition() == fanucBefore, "FANUC entered before retreat.");
                Require(c.TryExecute(CellRunCommand.Pause, out error), error); before = Full(l);
                await Task.Delay(150); Require(Full(l) == before && t.HandoffProductId == id && item.Owner == null, "Paused handoff advanced or changed evidence.");
                l.OutputA.position += Vector3.right * 9f;
                Require(c.TryExecute(CellRunCommand.Resume, out error), error); await Until(() => c.IsFaulted);
                Require(c.ControlledSequence == sq && c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Scara && fq.State == State.Idle &&
                    fq.GripCount == 0 && fq.ReleaseCount == 0 && t.HandoffProductId == id && item.Owner == null && !t.IsFaulted,
                    "Failed FANUC preflight lost SCARA permission or changed handoff evidence.");
                l.OutputA.position = outputPosition; await Ticks(2);
                Require(c.IsFaulted, "Repair implicitly cleared the automatic cycle fault.");
                Require(c.TryExecute(CellRunCommand.Reset, out error), error); before = Full(l); await Ticks(2);
                Require(c.State == CellRunState.Stopped && Full(l) == before, "Reset automatically advanced the handoff.");
                Require(c.TryExecute(CellRunCommand.Start, out error), error);
            }
            await Until(() =>
            {
                if (c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Scara)
                    Require(FanucPosition() == fanucBefore, "FANUC moved before SCARA completion.");
                if (c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Fanuc)
                {
                    Require(sq.State == State.Completed && sq.GripCount == 1 && sq.ReleaseCount == 1 &&
                        l.Scara.TryReadFkDiagnostics(out var fk, out _) && Vector3.Distance(fk.ActualPositionMetres, l.ScaraHandoffWaypoints.Retreat.PositionMetres) <= .002f,
                        "FANUC permission preceded SCARA release/retreat.");
                    if (!handoffObserved && window != null) ReadAutomaticUi(false, id, CellRunController.AutomaticTransferPhase.Fanuc);
                    handoffObserved = true;
                }
                if (item.Owner == fg) Require(fg.HeldItem == item && t.FanucProductId == id && t.HandoffProductId == null, "Actual/logical FANUC holding diverged.");
                return c.AutomaticPhase == CellRunController.AutomaticTransferPhase.Completed || c.IsFaulted;
            });
            var location = destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB;
            var output = destination == ProductDestination.A ? l.OutputA : l.OutputB;
            Require(!c.IsFaulted && c.State == CellRunState.Stopped && handoffObserved && sq.GripCount == 1 && sq.ReleaseCount == 1 &&
                fq.State == State.Completed && fq.GripCount == 1 && fq.ReleaseCount == 1 && item.Owner == null && fg.HeldItem == null &&
                t.TryGet(id, out var result) && result.Location == location && result.Destination == destination &&
                t.InputProductId == null && t.HandoffProductId == null && t.ScaraProductId == null && t.FanucProductId == null &&
                t.WaitingSnapshot().SequenceEqual(new[] { "AUTO-WAIT" }) && !t.IsFaulted &&
                Vector3.Distance(item.Body.position, output.position) <= .002f && RobotGripper.RotationError(item.Body.rotation, output.rotation) <= .5f,
                c.FaultReason ?? fq.LastError ?? "Automatic single-product completion failed.");
            if (window != null) ReadAutomaticUi(false, id, CellRunController.AutomaticTransferPhase.Completed);
            before = Full(l); Require(!c.TryExecute(CellRunCommand.Start, out _) && Full(l) == before, "Completed automatic cycle replayed.");
            Require(c.TryReleaseControl(out error) && c.AutomaticPhase == CellRunController.AutomaticTransferPhase.None &&
                c.AutomaticProductId == null && c.AutomaticDestination == null, error);
            if (window != null) ReadAutomaticUi(false, null, CellRunController.AutomaticTransferPhase.None);
            var report = new { passed = true, scenario, destination = destination.ToString(), commonInitialStarts = 1,
                scaraGrips = sq.GripCount, scaraReleases = sq.ReleaseCount, fanucGrips = fq.GripCount, fanucReleases = fq.ReleaseCount,
                handoffAfterRetreatObserved = handoffObserved, completed = 1, physicalWaiting = 1, explicitRemoved = scenario == 1 ? 1 : 0,
                registered = scenario == 1 ? 3 : 2, unconfirmed = 0, pauseAndExplicitFaultRecovery = scenario == 2,
                nextProductNotFed = true, uiViews, directClicks = 0, callbacksOrUiInputInjected = 0 };
            Directory.CreateDirectory("Logs/LW12A"); File.WriteAllText("Logs/LW12A/automatic-single-" + scenario + ".json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return JsonConvert.SerializeObject(report);
        }
        finally { l.OutputA.position = outputPosition; Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
        string FanucPosition() => JsonConvert.SerializeObject(new { p = l.Fanuc.Joints.Select(a => a.CurrentPosition), t = l.Fanuc.Joints.Select(a => a.TargetPosition), moving = l.Fanuc.IsTcpMoving });
        void ReadAutomaticUi(bool enabled, string id, CellRunController.AutomaticTransferPhase phase)
        {
            window.RefreshCell(); Check(window.rootVisualElement);
            var inspector = UnityEditor.Editor.CreateEditor(c);
            try { Check(inspector.CreateInspectorGUI()); }
            finally { UnityEngine.Object.DestroyImmediate(inspector); }
            void Check(VisualElement root)
            {
                var button = root.Q<Button>("cell-run-prepare-automatic-transfer"); var text = root.Q<Label>("cell-run-automatic-transfer").text;
                string expectedPhase = phase == CellRunController.AutomaticTransferPhase.None ? "미준비"
                    : phase == CellRunController.AutomaticTransferPhase.Scara ? "SCARA 운반/퇴피"
                    : phase == CellRunController.AutomaticTransferPhase.Fanuc ? "FANUC 배출" : "완료";
                Require(button != null && button.enabledInHierarchy == enabled && text.Contains(expectedPhase), "Automatic preparation eligibility/phase display mismatch.");
                if (id != null)
                    Require(phase == CellRunController.AutomaticTransferPhase.None ? root.Q<Label>("cell-run-transfer-target").text.Contains(id)
                        : text.Contains(id) && text.Contains("목적지 " + destination), "Automatic product preview mismatch.");
                if (phase == CellRunController.AutomaticTransferPhase.Completed)
                    Require(!root.Q<Button>("cell-run-start").enabledInHierarchy, "Completed automatic cycle exposes a replay button.");
                uiViews++;
            }
        }
    }

    public static async Task<string> RepeatTen()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var q = l.Fanuc.GetComponent<RobotWaypointSequence>(); var sq = l.Scara.GetComponent<RobotWaypointSequence>();
        var g = l.Fanuc.GetComponent<RobotGripper>(); var ids = new List<string> { "P-000001" }; var rows = new List<object>();
        float scale = Time.timeScale; int grips = 0, releases = 0, clearedCompletedObjects = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1);
            for (int i = 1; i < 10; i++)
            {
                string id = "FANUC-" + (i + 1).ToString("00"); ids.Add(id);
                var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * (i * .15f);
                item.Body.rotation = l.InputStop.rotation;
                Require(t.TryRegister(item, id, i % 2 == 0 ? ProductDestination.A : ProductDestination.B, out var e), e);
            }
            for (int i = 0; i < 10; i++)
            {
                string id = ids[i]; var destination = i % 2 == 0 ? ProductDestination.A : ProductDestination.B;
                Require(t.TryGetItem(id, out var item), "Original product missing.");
                if (i > 0)
                {
                    item.Body.position = l.InputStop.position; item.Body.rotation = l.InputStop.rotation; await Ticks(2);
                    Require(t.TryFeedNext(out var fed, out var e) && fed.Id == id, e ?? "FIFO feed order changed.");
                }
                Require(c.TryPrepareScaraTransfer(out var error) && c.TryExecute(CellRunCommand.Start, out error), error);
                await Until(() => sq.State == State.Completed || c.IsFaulted);
                Require(sq.State == State.Completed && sq.GripCount == 1 && sq.ReleaseCount == 1 && t.HandoffProductId == id,
                    sq.LastError ?? c.FaultReason ?? "SCARA fixture supply failed.");
                Require(c.TryExecute(CellRunCommand.Stop, out error) && c.TryReleaseControl(out error), error);
                Require(t.TryGet(id, out var expected) && expected.Destination == destination, "Handoff destination changed.");
                string products = Products(l), motion = Motion(l), scara = ScaraState(l);
                Require(c.TryPrepareFanucTransfer(destination, expected, out error), error);
                Require(c.State == CellRunState.Stopped && q.State == State.Paused && Products(l) == products && Motion(l) == motion,
                    "FANUC preparation changed products or issued motion.");
                Require(c.TryExecute(CellRunCommand.Start, out error), error);
                await Until(() =>
                {
                    if (item.Owner == g) Require(g.HeldItem == item && t.FanucProductId == id && t.HandoffProductId == null, "Physical/logical holding diverged.");
                    return q.State == State.Completed || c.IsFaulted;
                });
                var location = destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB;
                var output = destination == ProductDestination.A ? l.OutputA : l.OutputB;
                Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1 && item.Owner == null && g.HeldItem == null &&
                    t.TryGet(id, out var result) && result.Location == location && result.Destination == destination &&
                    t.InputProductId == null && t.ScaraProductId == null && t.HandoffProductId == null && t.FanucProductId == null &&
                    t.Count == 10 && !t.IsFaulted && !c.IsFaulted && ScaraState(l) == scara && t.WaitingSnapshot().SequenceEqual(ids.Skip(i + 1)),
                    q.LastError ?? c.FaultReason ?? "Completion changed identity, destination, FIFO, SCARA retreat or holding.");
                float positionError = Vector3.Distance(item.Body.position, output.position), rotationError = RobotGripper.RotationError(item.Body.rotation, output.rotation);
                Require(positionError <= .002f && rotationError <= .5f, "Actual output placement is outside tolerance.");
                var snapshot = t.Snapshot();
                Require(snapshot.Count(r => r.Location == ProductLocation.OutputA || r.Location == ProductLocation.OutputB) == i + 1 &&
                    snapshot.Count(r => r.Location == ProductLocation.Waiting) == 9 - i &&
                    snapshot.All(r => r.Location == ProductLocation.OutputA || r.Location == ProductLocation.OutputB ||
                        r.Location == ProductLocation.Waiting && t.TryGetItem(r.Id, out var original) && original.IsReady && original.Owner == null),
                    "Registered=completed+physical waiting accounting failed.");
                grips += q.GripCount; releases += q.ReleaseCount;
                rows.Add(new { cycle = i + 1, id, destination = destination.ToString(), grips = q.GripCount, releases = q.ReleaseCount,
                    positionErrorMetres = positionError, rotationErrorDegrees = rotationError, completed = i + 1, physicallyConfirmedResidual = 9 - i, unconfirmed = 0 });
                Require(c.TryExecute(CellRunCommand.Stop, out error) && c.TryReleaseControl(out error), error);
                // Test-only output clearance after actual placement was verified. Completion records remain terminal.
                if (i < 9) { UnityEngine.Object.Destroy(item.gameObject); await Ticks(2); clearedCompletedObjects++; }
            }
            var final = t.Snapshot();
            Require(final.Count(r => r.Location == ProductLocation.OutputA) == 5 && final.Count(r => r.Location == ProductLocation.OutputB) == 5 &&
                !t.IsFaulted && !c.IsFaulted && t.WaitingSnapshot().Count == 0, "Final ten-product accounting failed.");
            var report = new { passed = true, normalCycles = 10, completedA = 5, completedB = 5, automaticGrips = grips, automaticReleases = releases,
                registeredProducts = 10, explicitRemoved = 0, physicallyConfirmedResidual = 0, unconfirmedProducts = 0,
                clearedCompletedObjects, remainingCompletedObjects = 1, fixtureScaraSupplyCycles = 10, automaticTwoRobotDispatchImplemented = false,
                directClicks = 0, callbacksOrUiInputInjected = 0, rows };
            Write("fanuc-transfer-ten.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.normalCycles, report.completedA, report.completedB, grips, releases,
                report.physicallyConfirmedResidual, report.unconfirmedProducts, clearedCompletedObjects });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
    }

    public static async Task<string> UiPrepare(int destinationIndex)
    {
        Require(destinationIndex == 0 || destinationIndex == 1, "Use A=0 or B=1.");
        var destination = (ProductDestination)destinationIndex;
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var q = l.Fanuc.GetComponent<RobotWaypointSequence>(); float scale = Time.timeScale; int views = 0;
        var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show();
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1);
            Require(t.TryGet("P-000001", out var previous), "Initial record missing."); Read(false, null);
            var item = await SupplyHandoff(l, destination, false);
            Require(t.TryGet(item.ProductId, out var displayed), "Displayed handoff record missing."); Read(true, item.ProductId);
            await Task.Delay(150);
            var advanced = w.rootVisualElement.Q<Foldout>("cell-run-individual-tests");
            Require(advanced != null && !advanced.value && advanced.Contains(w.rootVisualElement.Q<Button>("cell-run-prepare-fanuc-transfer")), "Individual FANUC preparation is not in the advanced section.");
            string before = Full(l);
            Require(!c.TryPrepareFanucTransfer(destination, previous, out _) && Full(l) == before, "Previous product record changed a newer handoff.");
            string products = Products(l), motion = Motion(l), physical = Physical(l);
            Require(c.TryPrepareFanucTransfer(displayed.Destination, displayed, out var e), e);
            Require(c.State == CellRunState.Stopped && q.State == State.Paused && q.FanucDestination == destination &&
                Products(l) == products && Motion(l) == motion && Physical(l) == physical, "Preparation emitted motion or changed product evidence."); Read(false, item.ProductId);
            before = Full(l); Require(!c.TryPrepareFanucTransfer(destination, displayed, out _) && Full(l) == before, "Duplicate preparation changed the path.");
            Require(c.TryExecute(CellRunCommand.Start, out e), e); Read(false, item.ProductId);
            await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1 && item.Owner == null &&
                t.TryGet(item.ProductId, out var result) && result.Destination == destination &&
                result.Location == (destination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB), q.LastError ?? c.FaultReason);
            Read(false, null);
            var report = new { passed = true, destination = destination.ToString(), uiViews = views, automaticGrips = q.GripCount,
                automaticReleases = q.ReleaseCount, preparationWithoutMotion = true, previousRecordAndDuplicateRejected = true,
                directClicks = 0, callbacksOrUiInputInjected = 0 };
            Write("fanuc-ui-prepare-" + destination.ToString().ToLowerInvariant() + ".json", report);
            return JsonConvert.SerializeObject(report);
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }

        void Read(bool enabled, string id)
        {
            w.RefreshCell(); Check(w.rootVisualElement);
            var inspector = UnityEditor.Editor.CreateEditor(c);
            try { Check(inspector.CreateInspectorGUI()); }
            finally { UnityEngine.Object.DestroyImmediate(inspector); }
            void Check(VisualElement root)
            {
                var button = root.Q<Button>("cell-run-prepare-fanuc-transfer"); var text = root.Q<Label>("cell-run-fanuc-transfer-target").text;
                Require(button != null && button.enabledInHierarchy == enabled &&
                    (id == null ? text.Contains("인계 상품 없음") : text.Contains(id) && text.Contains("목적지 " + destination) && text.Contains(destination + " 출구")),
                    "FANUC preparation UI disagrees with handoff eligibility/identity/destination."); views++;
            }
        }
    }

    private static bool CanPrepare(CellRunController c, ProductDestination destination, out string error)
        => destination == ProductDestination.A ? c.CanPrepareFanucTransferToA(out error) : c.CanPrepareFanucTransfer(destination, out error);
    private static bool Prepare(CellRunController c, ProductDestination destination, out string error)
        => destination == ProductDestination.A ? c.TryPrepareFanucTransferToA(out error) : c.TryPrepareFanucTransfer(destination, out error);
    private static bool Prepare(CellRunController c, ProductDestination destination, ProductRecord expected, out string error)
        => destination == ProductDestination.A ? c.TryPrepareFanucTransferToA(expected, out error) : c.TryPrepareFanucTransfer(destination, expected, out error);

    private static async Task<GrippableItem> SupplyHandoff(LineWatchCellLayout l, ProductDestination destination, bool waiting)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); var q = l.Scara.GetComponent<RobotWaypointSequence>();
        await Until(() => t.Count == 1); string id = "P-000001";
        if (destination == ProductDestination.B)
        {
            Require(t.TryRemove(id, ProductLocation.Input, out var e), e); id = "B-ONLY";
            var candidate = UnityEngine.Object.Instantiate(l.ProductPrefab); candidate.Body.position = l.InputStop.position; candidate.Body.rotation = l.InputStop.rotation;
            Require(t.TryRegister(candidate, id, destination, out e) && t.TryFeedNext(out _, out e), e);
        }
        if (waiting) { Add(l, "WAIT-A", .4f); Add(l, "WAIT-B", .8f); }
        Require(t.TryGetItem(id, out var item), "Fixture input missing.");
        Require(c.TryPrepareScaraTransfer(out var error) && c.TryExecute(CellRunCommand.Start, out error), error);
        await Until(() => q.State == State.Completed || c.IsFaulted);
        Require(q.State == State.Completed && t.HandoffProductId == id && item.Owner == null, q.LastError ?? c.FaultReason);
        Require(c.TryExecute(CellRunCommand.Stop, out error) && c.TryReleaseControl(out error), error);
        return item;
    }
    private static void Add(LineWatchCellLayout l, string id, float offset)
    { var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset; Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e); }
    private static void SetPick(LineWatchCellLayout l, RobotWaypointSet set)
    { var so = new SerializedObject(l); so.FindProperty("fanucHandoffWaypoints").objectReferenceValue = set; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetScaraTemplate(LineWatchCellLayout l, RobotWaypointSet set)
    { var so = new SerializedObject(l); so.FindProperty("scaraHandoffWaypoints").objectReferenceValue = set; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetRetreat(RobotWaypointSet set, Vector3 p)
    { var so = new SerializedObject(set); so.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = p; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static LineWatchCellLayout Layout()
    { Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh unpaused Play."); var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>(); Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch."); return l; }
    private static string Products(LineWatchCellLayout l)
    { var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(), slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId } }); }
    private static string Physical(LineWatchCellLayout l) => JsonConvert.SerializeObject(UnityEngine.Object.FindObjectsByType<GrippableItem>(FindObjectsInactive.Include, FindObjectsSortMode.InstanceID).Select(i => new { id = Ref(i), i.ProductId, owner = Ref(i.Owner), p = V(i.Body.position), r = Q(i.Body.rotation), active = i.gameObject.activeInHierarchy }));
    private static string ScaraState(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { p = new[] { l.Scara.Axis1.CurrentPosition, l.Scara.Axis2.CurrentPosition, l.Scara.Axis3.CurrentPosition, l.Scara.Axis4.CurrentPosition }, t = new[] { l.Scara.Axis1.TargetPosition, l.Scara.Axis2.TargetPosition, l.Scara.Axis3.TargetPosition, l.Scara.Axis4.TargetPosition }, moving = l.Scara.IsTrajectoryMoving });
    private static string Motion(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { scara = ScaraState(l), sv = l.Scara.CommandVersion, fv = l.Fanuc.CommandVersion,
        p = l.Fanuc.Joints.Select(a => a.CurrentPosition), t = l.Fanuc.Joints.Select(a => a.TargetPosition), moving = l.Fanuc.IsTcpMoving,
        held = new[] { Ref(l.Scara.GetComponent<RobotGripper>().HeldItem), Ref(l.Fanuc.GetComponent<RobotGripper>().HeldItem) } });
    private static string Data(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { products = Products(l), physical = Physical(l), motion = Motion(l),
        paths = new[] { l.Scara.GetComponent<RobotWaypointSequence>(), l.Fanuc.GetComponent<RobotWaypointSequence>() }.Select(q => new { q.StepIndex, q.IsCellTransfer, q.IsFanucTransfer, q.ProgramRevision, q.GripCount, q.ReleaseCount, q.LastArrivedStepIndex }) });
    private static string Full(LineWatchCellLayout l)
    { var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { data = Data(l), c.State, c.FaultReason, c.LastAcknowledgedFaultReason, c.HasControlledSequence, c.AutomaticPhase, c.AutomaticProductId, c.AutomaticDestination, c.IsAutomaticContinuous, controlled = Ref(c.ControlledSequence), t.IsFaulted, tracker = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason,
        paths = new[] { l.Scara.GetComponent<RobotWaypointSequence>(), l.Fanuc.GetComponent<RobotWaypointSequence>() }.Select(q => new { q.State, q.LastError, q.LastAcknowledgedFaultReason }) }); }
    private static int Ref(UnityEngine.Object o) => ReferenceEquals(o, null) ? 0 : o.GetInstanceID();
    private static float[] V(Vector3 v) => new[] { v.x, v.y, v.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }.SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", ""); }
    private static void Check(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification modified sources/assets.");
    private static async Task Ticks(int count) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= count * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 25, "Simulation wait timed out."); await Task.Delay(1); } }
    private static void Write(string file, object report) { Directory.CreateDirectory("Logs/LW12"); File.WriteAllText("Logs/LW12/" + file, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
