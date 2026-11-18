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

public static class VerifyLw11Completion
{
    public static async Task<string> ReproduceStaleLoss()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        await Until(() => t.Count == 1); Require(t.TryGet("P-000001", out var old), "Missing old record.");
        Require(t.TryGetItem(old.Id, out var item), "Missing original."); t.enabled = false; t.enabled = true;
        Require(t.TryReturnToQueue(old, "같은 ID 재투입 재현", out var e), e);
        Require(c.TryExecute(CellRunCommand.Reset, out e) && t.TryFeedNext(out _, out e), e);
        Require(t.TryGet(old.Id, out var current) && !ReferenceEquals(old, current) && old.Location == current.Location, "Fixture did not return to same location.");
        UnityEngine.Object.Destroy(item.gameObject); await Until(() => !t.TryGetItem(old.Id, out _) && c.IsFaulted && t.IsFaulted);
        bool oldUiEligible = t.CanConfirmDestroyedLoss(old.Id, old.Location, "이전 선택의 사유", out e);
        var report = new { reproduced = oldUiEligible, sameIdAndLocationButNewRecord = true, locationOnlySelectionEligible = oldUiEligible, receiptCreated = false };
        Write("completion-stale-loss-before.json", report); return JsonConvert.SerializeObject(report);
    }

    public static async Task<string> Requeue(int scenario)
    {
        Require(scenario >= 0 && scenario <= 3, "Use Input/Scara/Handoff/Fanuc scenarios 0..3.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var fg = l.Fanuc.GetComponent<RobotGripper>();
        var hashes = Sources(); float scale = Time.timeScale; int rejects = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); Add(l, "WAIT-A", .4f); Add(l, "WAIT-B", .8f);
            Require(t.TryGetItem("P-000001", out var item), "Missing input object.");
            Require(t.TryGet("P-000001", out var original), "Missing input record.");
            Reject(original, "healthy cell");
            if (scenario > 0)
            {
                await Move(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
                if (scenario >= 2)
                {
                    await Move(sg, item, l.HandoffPoint); Require(sg.TryRelease(out e), e);
                    var retreat = l.ScaraHandoffWaypoints.Retreat;
                    Require(l.Scara.TryMoveTcp(retreat.PositionMetres, Yaw(retreat.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out e), e);
                    await Until(() => sg.IsStopped);
                    if (scenario == 3) { await Move(fg, item, l.HandoffPoint); Require(fg.TryGrip(item, out e), e); }
                }
            }
            Require(t.TryGet("P-000001", out var selected), "Record missing.");
            var g = scenario == 3 ? fg : sg; var q = g.GetComponent<RobotWaypointSequence>();
            if (scenario == 1) { Require(q.TryStart(out var e), e); q.Pause(); Require(c.TryTakeControl(q, out e), e); }
            t.enabled = false; t.enabled = true;
            if (scenario == 1)
            {
                Reject(selected, "held and pending"); Require(c.TryCancelProgram(q, "원래 보유 상실 복구 전 경로 종료", out var e), e);
                Reject(selected, "actual holding");
            }
            if (scenario == 3) Reject(selected, "actual FANUC holding");
            if (scenario == 1 || scenario == 3) { item.gameObject.SetActive(false); item.gameObject.SetActive(true); }
            else item.Body.position += Vector3.right * .02f;
            await Ticks(2); Time.timeScale = 1;
            Require(c.IsFaulted && t.IsFaulted && item.Owner == null && sg.HeldItem == null && fg.HeldItem == null, "Lost-owner fixture invalid.");
            string faults = Faults(l), physical = Physical(l), motion = Motion(l), before = Full(l);
            for (int i = 0; i < 100; i++) Require(t.CanReturnToQueue(selected, "원래 실물 확인 후 대기 복귀", out var e), e);
            Require(Full(l) == before, "Requeue queries changed evidence.");
            Reject(null, "no target"); Reject(selected, "no reason", null); Reject(selected, "blank reason", " ");
            if (scenario > 0) Reject(original, "stale record");
            t.TryGet("WAIT-A", out var waiting); Reject(waiting, "already waiting");
            Time.timeScale = 0; Reject(selected, "time stopped"); Time.timeScale = 1;
            EditorApplication.isPaused = true; Reject(selected, "Editor paused"); EditorApplication.isPaused = false;
            c.enabled = false; Reject(selected, "manager disabled"); c.enabled = true;
            t.enabled = false; Reject(selected, "tracker disabled"); t.enabled = true;
            item.gameObject.SetActive(false); Reject(selected, "original inactive"); item.gameObject.SetActive(true); await Ticks(2);
            Require(c.IsFaulted && t.IsFaulted && t.QueueRecoverySnapshot().Length == 0, "Reactivation acknowledged/requeued.");
            Require(t.TryReturnToQueue(selected, "원래 실물 확인 후 대기 복귀", out var error), error);
            Require(Faults(l) == faults && Physical(l) == physical && Motion(l) == motion && t.Count == 3 &&
                t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B", "P-000001" }) &&
                t.InputProductId == null && t.ScaraProductId == null && t.HandoffProductId == null && t.FanucProductId == null,
                "Requeue changed physical ownership, other FIFO order, counts or faults.");
            var receipt = t.QueueRecoverySnapshot().Single();
            Require(ReferenceEquals(receipt.Previous, selected) && receipt.Result.Id == selected.Id && receipt.Result.Destination == selected.Destination &&
                receipt.Result.Location == ProductLocation.Waiting && !string.IsNullOrEmpty(receipt.Reason), "Recovery receipt lost identity/cause.");
            Reject(selected, "stale duplicate");
            before = Full(l); await Ticks(3); Require(Full(l) == before, "Requeue automatically acknowledged or fed.");
            Require(c.TryExecute(CellRunCommand.Reset, out error), error); Require(c.State == CellRunState.Stopped && !t.IsFaulted, "Separate acknowledgement failed.");
            foreach (string id in new[] { "WAIT-A", "WAIT-B", "P-000001" })
            {
                Require(t.TryGetItem(id, out var next), "Original survivor missing.");
                next.Body.position = l.InputStop.position; next.Body.rotation = l.InputStop.rotation; await Ticks(2);
                Require(t.TryFeedNext(out var fed, out error) && fed.Id == id, error ?? "FIFO order changed.");
                Require(t.TryRemove(id, ProductLocation.Input, out error), error);
            }
            Require(t.QueueRecoverySnapshot().Single() == receipt && t.Count == 3, "Later operations erased recovery evidence.");
            Check(hashes); var report = new { passed = true, scenario, eligibilityQueriesPreserved = 100, rejectedRequestsPreserved = rejects,
                confirmedQueueReturns = 1, preservedFifoThenRefeeds = 3, separateAcknowledgement = true, faultsAndPhysicalPreserved = true,
                directClicks = 0, callbackOrUiInputInjection = 0, sourcesPreserved = hashes.Count, hashes };
            Write("completion-requeue-" + scenario + ".json", report); return JsonConvert.SerializeObject(new { report.passed, scenario, rejects });
        }
        finally { EditorApplication.isPaused = false; Time.timeScale = scale; c.enabled = true; t.enabled = true; c.TryExecute(CellRunCommand.Stop, out _); }
        void Reject(ProductRecord record, string name, string reason = "검증 중 사유")
        { string before = Full(l); Require(!t.CanReturnToQueue(record, reason, out _) && !t.TryReturnToQueue(record, reason, out var e) && !string.IsNullOrEmpty(e) && Full(l) == before, "Rejection changed evidence: " + name); rejects++; }
    }

    public static async Task<string> Scenario()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = l.Scara.GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>();
        var hashes = Sources(); float scale = Time.timeScale; int rejects = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1);
            for (int cycle = 0; cycle < 5; cycle++)
            {
                string id = cycle == 0 ? "P-000001" : "SCENARIO-" + cycle;
                if (cycle > 0)
                {
                    var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position; item.Body.rotation = l.InputStop.rotation;
                    string before = Full(l); Require(!t.TryRegister(item, "P-000001", ProductDestination.A, out _) && !item.IsTracked && Full(l) == before, "Prior ID reused after scenario reset."); rejects++;
                    Require(t.TryRegister(item, id, ProductDestination.B, out var e) && t.TryFeedNext(out _, out e), e);
                }
                Reject(t.ScenarioNumber, "nonterminal product");
                Require(t.TryRemove(id, ProductLocation.Input, out var error), error);
                var lab = g.GetComponent<RobotGripperLab>(); Require(lab.TryPrepare(out error), error);
                if (cycle == 0)
                {
                    Require(q.TryStartPickPlace(g, lab.TrialItem, out error), error); q.Pause();
                    uint stale = q.ProgramRevision - 1; t.enabled = false; t.enabled = true;
                    string before = Full(l);
                    Require(!c.TryCancelProgram(q, stale, "오래된 뷰의 종료 요청", out _) && Full(l) == before, "Stale selection cancelled a replacement program."); rejects++;
                    Reject(t.ScenarioNumber, "faulted cell");
                    Require(c.TryCancelProgram(q, q.ProgramRevision, "현재 경로의 명시 종료", out error), error);
                    Require(c.TryExecute(CellRunCommand.Reset, out error), error);
                    Require(lab.TryMoveToWork(out error), error); await Until(() => g.IsStopped);
                    Require(g.TryGrip(lab.TrialItem, out error), error); Reject(t.ScenarioNumber, "actual holding with no pending path");
                    Require(g.TryRelease(out error), error); Require(lab.TryPrepare(out error), error);
                }
                Require(q.TryStartPickPlace(g, lab.TrialItem, out error), error); q.Pause(); Require(c.TryTakeControl(q, out error), error);
                Reject(t.ScenarioNumber, "unfinished adopted path");
                uint revision = q.ProgramRevision;
                Require(c.TryExecute(CellRunCommand.Start, out error), error); await Until(() => q.State == State.Completed || c.IsFaulted);
                Require(q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1, q.LastError ?? "Pick/place did not complete.");
                Reject(t.ScenarioNumber, "running cell"); Require(c.TryExecute(CellRunCommand.Stop, out error), error);
                Reject(t.ScenarioNumber, "no reason", null); Reject(t.ScenarioNumber, "blank reason", " ");
                Reject(t.ScenarioNumber - 1, "stale scenario");
                Time.timeScale = 0; Reject(t.ScenarioNumber, "time stopped"); Time.timeScale = 5;
                EditorApplication.isPaused = true; Reject(t.ScenarioNumber, "Editor paused"); EditorApplication.isPaused = false;
                string beforeReset = Full(l); for (int i = 0; i < 100; i++) Require(c.CanResetScenario("기록 보존 초기화", out error), error);
                Require(Full(l) == beforeReset, "Initialization query changed evidence.");
                string records = Records(l), physical = Physical(l), motion = Motion(l); int number = t.ScenarioNumber;
                var faults = Faults(l); var cancellations = q.CancellationSnapshot();
                Require(c.TryResetScenario(number, "기록 보존 초기화", out error), error);
                Require(t.ScenarioNumber == number + 1 && t.CurrentScenarioSnapshot().Length == 0 && t.Count == cycle + 1 &&
                    t.ScenarioSnapshot().Length == cycle + 1 && t.ScenarioSnapshot().Last().Products.Single().Id == id &&
                    c.State == CellRunState.Stopped && !c.HasControlledSequence && c.ControlledSequence == null && q.State == State.Idle &&
                    l.Fanuc.GetComponent<RobotWaypointSequence>().State == State.Idle && q.StepIndex == 0 && q.GripCount == 0 && q.ReleaseCount == 0 &&
                    q.ProgramRevision != revision && Records(l) == records && Physical(l) == physical && Motion(l) == motion && Faults(l) == faults,
                    "Scenario reset lost history, changed physical/motor state, or spawned/started a program.");
                var old = q.ScenarioResetSnapshot().Last();
                Require(old.PreviousState == State.Completed && old.GripCount == 1 && old.ReleaseCount == 1 && old.ProgramRevision == revision &&
                    q.CancellationSnapshot().SequenceEqual(cancellations), "Initialization erased completed actions/cancellations.");
                Reject(number, "stale request after reset"); Reject(t.ScenarioNumber, "duplicate empty reset");
                string after = Full(l); await Ticks(3); Require(Full(l) == after, "Initialization automatically generated a sample or started.");
            }
            Require(t.Count == 5 && t.CurrentScenarioSnapshot().Length == 0 && t.ScenarioNumber == 6, "Five-cycle accounting mismatch.");
            Check(hashes); var report = new { passed = true, resetCycles = 5, completedPickPlacePrograms = 5, eligibilityQueriesPreserved = 500,
                rejectedRequestsPreserved = rejects, cumulativeProductHistory = t.Count, currentProducts = 0, nextScenario = t.ScenarioNumber,
                oldIdsRemainReserved = true, completedActionHistoryPreserved = true, physicalAndMotorStateUnchanged = true, generatedSamplesAtReset = 0,
                directClicks = 0, callbackOrUiInputInjection = 0, sourcesPreserved = hashes.Count, hashes };
            Write("completion-scenario.json", report); return JsonConvert.SerializeObject(new { report.passed, report.resetCycles, report.rejectedRequestsPreserved });
        }
        finally { EditorApplication.isPaused = false; Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
        void Reject(int number, string name, string reason = "검증 중 사유" )
        { string before = Full(l); Require(!c.CanResetScenario(number, reason, out _) && !c.TryResetScenario(number, reason, out var e) && !string.IsNullOrEmpty(e) && Full(l) == before, "Reset rejection changed state: " + name); rejects++; }
    }

    public static async Task<string> Boundaries()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); var hashes = Sources();
        await Until(() => t.Count == 1); Add(l, "WAIT-A", .4f); Add(l, "WAIT-B", .8f);
        Require(t.TryGetItem("P-000001", out var item), "Missing initial item.");
        Require(t.TryGetItem("WAIT-A", out var waiting), "Missing second unresolved item.");
        Require(t.TryGet("P-000001", out var old), "Missing initial record.");
        t.enabled = false; t.enabled = true; waiting.gameObject.SetActive(false);
        Require(t.TryReturnToQueue(old, "복수 미확정 중 원래 상품 대기 복귀", out var error), error);
        string before = Full(l);
        Require(!c.TryExecute(CellRunCommand.Reset, out _) && Full(l) == before && t.IsFaulted && c.IsFaulted,
            "One recovery normalized a different unavailable product.");
        waiting.gameObject.SetActive(true); await Ticks(2);
        Require(c.IsFaulted && t.IsFaulted, "Physical repair implicitly acknowledged multiple faults.");
        Require(c.TryExecute(CellRunCommand.Reset, out error), error);
        Require(t.TryRemove("WAIT-A", ProductLocation.Waiting, out error) && t.TryRemove("WAIT-B", ProductLocation.Waiting, out error), error);
        Require(t.TryFeedNext(out var current, out error), error); t.enabled = false; t.enabled = true;
        before = Full(l);
        Require(current.Location == old.Location && !ReferenceEquals(current, old) && !t.TryReturnToQueue(old, "같은 위치의 오래된 선택", out _) && Full(l) == before,
            "Old selection was silently updated to a newer record at the same logical location.");
        Require(t.CanReturnToQueue(current, "새로 선택한 원래 실물", out error), error);
        var originalObject = item.gameObject; var body = item.Body; var position = body.position;
        UnityEngine.Object.Destroy(item); await Until(() => !t.TryGetItem("P-000001", out _));
        var replacement = UnityEngine.Object.Instantiate(l.ProductPrefab); replacement.Body.position = l.InputStop.position;
        await Ticks(2); before = Full(l);
        Require(!t.TryReturnToQueue(current, "컴포넌트만 삭제", out _) && !t.TryConfirmDestroyedLoss(current.Id, current.Location, "남아 있는 원본", out _) &&
            !t.TryRegister(replacement, current.Id, current.Destination, out _) && !c.TryExecute(CellRunCommand.Reset, out _) && !c.TryResetScenario("초기화 우회", out _) && Full(l) == before,
            "Component loss/replacement bypassed original-object evidence, fault acknowledgement or scenario eligibility.");
        c.enabled = false; c.enabled = true; t.enabled = false; t.enabled = true; await Ticks(2);
        Require(Full(l) == before && originalObject != null && body != null && Vector3.Distance(body.position, position) < .000001f && !replacement.IsTracked,
            "Reactivation or a replacement repaired the unknown original implicitly.");
        UnityEngine.Object.Destroy(originalObject); await Until(() => originalObject == null);
        before = Full(l);
        Require(!t.CanConfirmDestroyedLoss(old, "오래된 손실 선택", out _) && !t.TryConfirmDestroyedLoss(old, "오래된 손실 선택", out _) &&
            t.CanConfirmDestroyedLoss(current, "새 기록으로 원본 삭제 확인", out error) && Full(l) == before,
            "Same-location stale loss selection was accepted or current evidence could not be confirmed.");
        string causes = Faults(l);
        Require(t.TryConfirmDestroyedLoss(current, "새 기록으로 원본 삭제 확인", out error) && Faults(l) == causes &&
            t.TryGet(current.Id, out var lost) && lost.Location == ProductLocation.Lost && lost.LostFrom == current.Location && t.Count == 3,
            error ?? "Current-record loss confirmation changed fault/ID history.");
        Check(hashes); var report = new { passed = true, anotherUnresolvedProductRemainsFaulted = true, sameLocationStaleRecordRejected = true,
            componentLossAndReplacementRejected = true, resetCannotHideUnknown = true, reactivationDoesNotRecover = true, staleLossRecordRejected = true, currentRecordLossConfirmed = 1, rejectedRequestsPreserved = 8,
            sourcesPreserved = hashes.Count, directClicks = 0, callbackOrUiInputInjection = 0, hashes };
        Write("completion-boundaries.json", report); return JsonConvert.SerializeObject(new { report.passed, report.rejectedRequestsPreserved });
    }

    public static async Task<string> Ui(bool playing)
    {
        Require(EditorApplication.isPlaying == playing, "Wrong play state.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>(); var c = l.GetComponent<CellRunController>(); var hashes = Sources();
        var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
        var editor = UnityEditor.Editor.CreateEditor(c); var inspector = editor.CreateInspectorGUI();
        try
        {
            await Task.Delay(120); Forms(window.rootVisualElement, playing); Forms(inspector, playing);
            if (playing)
            {
                c.enabled = false; await Task.Delay(150); Forms(window.rootVisualElement, false); Forms(editor.CreateInspectorGUI(), false);
                c.enabled = true; await Task.Delay(150); Forms(window.rootVisualElement, true); Forms(editor.CreateInspectorGUI(), true);
                Require(c.IsFaulted, "UI/readiness reactivation cleared a fault.");
            }
            Check(hashes); var report = new { passed = true, playing, forms = 6, defaultCollapsed = true, automaticSelection = false,
                directClicks = 0, callbacksInjected = 0, uiValuesAssignedByVerifier = 0, actualInputAndCallbackPending = true, sourcesPreserved = hashes.Count, hashes };
            Write("completion-ui-" + (playing ? "play" : "edit") + ".json", report); return JsonConvert.SerializeObject(new { report.passed, report.forms, playing });
        }
        finally { UnityEngine.Object.DestroyImmediate(editor); }
    }

    private static void Forms(VisualElement root, bool ready)
    {
        foreach (string prefix in new[] { "cell-cancel", "cell-requeue", "cell-scenario" })
        {
            Require(!root.Q<Foldout>(prefix + "-panel").value && !root.Q<Button>(prefix + "-confirm").enabledInHierarchy &&
                root.Q<TextField>(prefix + "-reason").value == "" && root.Q<TextField>(prefix + "-reason").enabledInHierarchy == ready, "Wrong form defaults/permissions: " + prefix);
            if (prefix != "cell-scenario") Require(root.Q<DropdownField>(prefix + "-target").value == "대상을 선택하세요" && root.Q<DropdownField>(prefix + "-target").enabledInHierarchy == ready, "Form selected a target automatically.");
        }
    }
    private static async Task Move(RobotGripper g, GrippableItem item, Transform station)
    {
        var point = item.GetGripPoint(g.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc);
        var rotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = station.TransformPoint(point.localPosition) - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        var local = Quaternion.Inverse(g.RobotFrame.rotation) * rotation; string error;
        bool ok = g.Scara != null ? g.Scara.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), Yaw(local), ScaraJointLab.IkSelection.PositiveElbow, out _, out error)
            : g.Fanuc.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), local, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error);
        Require(ok, error); await Until(() => g.IsStopped); await Ticks(2);
    }
    private static void Add(LineWatchCellLayout l, string id, float offset)
    { var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset; Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e); }
    private static LineWatchCellLayout Layout()
    { Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh Play."); var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>(); Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch."); return l; }
    private static string Records(LineWatchCellLayout l)
    { var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(t.Snapshot()); }
    private static string Faults(LineWatchCellLayout l)
    { var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.LastAcknowledgedFaultReason, t.IsFaulted, tracker = t.FaultReason, archive = t.LastAcknowledgedFaultReason }); }
    private static string Physical(LineWatchCellLayout l) => JsonConvert.SerializeObject(UnityEngine.Object.FindObjectsByType<GrippableItem>(FindObjectsInactive.Include, FindObjectsSortMode.InstanceID).Select(i => new { id = i.GetInstanceID(), i.ProductId, active = i.gameObject.activeInHierarchy, owner = Ref(i.Owner), p = V(i.Body.position), r = Q(i.Body.rotation) }));
    private static string Motion(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { s = l.Scara.CommandVersion, f = l.Fanuc.CommandVersion,
        current = new[] { l.Scara.Axis1.CurrentPosition, l.Scara.Axis2.CurrentPosition, l.Scara.Axis3.CurrentPosition, l.Scara.Axis4.CurrentPosition }.Concat(l.Fanuc.Joints.Select(j => j.CurrentPosition)),
        target = new[] { l.Scara.Axis1.TargetPosition, l.Scara.Axis2.TargetPosition, l.Scara.Axis3.TargetPosition, l.Scara.Axis4.TargetPosition }.Concat(l.Fanuc.Joints.Select(j => j.TargetPosition)),
        held = new[] { Ref(l.Scara.GetComponent<RobotGripper>().HeldItem), Ref(l.Fanuc.GetComponent<RobotGripper>().HeldItem) } });
    private static string Full(LineWatchCellLayout l)
    {
        var t = l.GetComponent<CellProductTracker>(); var c = l.GetComponent<CellRunController>();
        return JsonConvert.SerializeObject(new { records = Records(l), fifo = t.WaitingSnapshot(), slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            t.ScenarioNumber, current = t.CurrentScenarioSnapshot(), scenarios = t.ScenarioSnapshot(), recoveries = t.QueueRecoverySnapshot(), faults = Faults(l), physical = Physical(l), motion = Motion(l),
            c.HasControlledSequence, controlled = Ref(c.ControlledSequence), paths = new[] { l.Scara.GetComponent<RobotWaypointSequence>(), l.Fanuc.GetComponent<RobotWaypointSequence>() }.Select(q => new { q.State, q.ProgramRevision, q.StepIndex, q.GripCount, q.ReleaseCount, q.LastError, q.LastAcknowledgedFaultReason, cancel = q.CancellationSnapshot().Length, resets = q.ScenarioResetSnapshot().Length }) });
    }
    private static int Ref(UnityEngine.Object o) => ReferenceEquals(o, null) ? 0 : o.GetInstanceID();
    private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
    private static float[] V(Vector3 v) => new[] { v.x, v.y, v.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }.SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", ""); }
    private static void Check(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification changed sources.");
    private static async Task Ticks(int count) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= count * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string file, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + file, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
