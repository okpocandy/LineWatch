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

public static class VerifyCellFaultRecovery
{
    public static async Task<object> Cycles()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var hashes = Sources(); float scale = Time.timeScale; int recovered = 0, queries = 0, rejected = 0;
        var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1 || t.IsFaulted); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
            string products = Products(l);
            foreach (var g in new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() })
            {
                var lab = g.GetComponent<RobotGripperLab>(); var q = g.GetComponent<RobotWaypointSequence>();
                Require(lab.TryPrepare(out var e) && lab.TryStartPickPlace(out e), e);
                await Until(() => q.GripCount == 1 && q.StepIndex == 2 || q.State == State.Faulted);
                Require(q.IsBusy && g.HeldItem == lab.TrialItem, q.LastError);
                q.Pause(); Require(c.TryTakeControl(q, out e), e);
                Time.timeScale = .1f; Require(c.TryExecute(CellRunCommand.Start, out e), e); await Ticks(1);
                c.enabled = false; c.enabled = true; Time.timeScale = 1; await Ticks(2);
                Require(c.IsFaulted && !t.IsFaulted && c.ControlledSequence == q && q.State == State.Paused, "Cell-only fault fixture failed.");
                string reason = c.FaultReason, before = Conservation(l), full = Full(l); var heldPose = lab.TrialItem.Body.position;
                for (int i = 0; i < 100; i++)
                { Require(c.CanExecute(CellRunCommand.Reset, out e), e); queries++; }
                Require(Full(l) == full && c.IsFaulted, "Eligibility reading recovered/mutated the cell.");
                await Task.Delay(200);
                var root = window.rootVisualElement;
                Require(root.Q<Button>("cell-run-reset").enabledInHierarchy && root.Q<Button>("cell-run-reset").text.Contains("고장 해제") &&
                    !root.Q<Button>("cell-run-start").enabledInHierarchy && root.Q<HelpBox>("cell-run-fault").text.Contains(reason), "Faulted recovery UI incorrect.");
                Require(c.TryExecute(CellRunCommand.Reset, out e), e);
                Require(c.State == CellRunState.Stopped && c.FaultReason == null && c.LastAcknowledgedFaultReason == reason && Conservation(l) == before,
                    "Acknowledgement changed data/path/commands or lost cause history.");
                Require(c.HasControlledSequence && c.ControlledSequence == q && q.State == State.Paused && q.StepIndex == 2 &&
                    q.GripCount == 1 && q.ReleaseCount == 0 && g.HeldItem == lab.TrialItem && lab.TrialItem.Owner == g, "Acknowledgement dropped/replayed holding.");
                await Ticks(3); await Task.Delay(150);
                Require(c.State == CellRunState.Stopped && q.State == State.Paused && Vector3.Distance(heldPose, lab.TrialItem.Body.position) < .00001f,
                    "Acknowledgement automatically restarted/moved the robot.");
                Require(!root.Q<Button>("cell-run-reset").enabledInHierarchy && root.Q<Button>("cell-run-start").enabledInHierarchy &&
                    root.Q<HelpBox>("cell-run-fault").text.Contains("최근 명시 해제"), "Stopped acknowledgement UI/history incorrect.");
                full = Full(l);
                Require(!c.TryExecute(CellRunCommand.Reset, out _) && !q.TryResume(out _) && !g.TryRelease(out _) && Full(l) == full,
                    "Duplicate acknowledgement or individual command bypassed retained ownership."); rejected += 3;
                Time.timeScale = 5; Require(c.TryExecute(CellRunCommand.Start, out e), e);
                await Until(() => q.State == State.Completed || c.IsFaulted || q.State == State.Faulted);
                Require(q.State == State.Completed && !c.IsFaulted && q.GripCount == 1 && q.ReleaseCount == 1 && g.HeldItem == null && Products(l) == products,
                    q.LastError ?? c.FaultReason ?? "Recovery repeated grip/release or changed records.");
                Require(c.TryExecute(CellRunCommand.Stop, out e) && c.TryReleaseControl(out e), e); recovered++;
            }
            CheckSources(hashes);
            var report = new { passed = true, verifiedUtc = DateTime.UtcNow, explicitAcknowledgements = recovered, completedCycles = recovered,
                eligibilityQueriesPreserved = queries, rejectedRequestsPreserved = rejected, registeredProducts = 3, physicallyConfirmedResidual = 3,
                unconfirmedProducts = 0, motionCommandsAtAcknowledgement = 0, directButtonClicks = 0, callbacksInjected = 0, sourcesPreserved = hashes.Count, hashes,
                scope = "Cell-only acknowledgement while an untracked trial is held. No tracker/path-fault recovery or data-clearing reset." };
            Write("cell-fault-recovery-cycles.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.explicitAcknowledgements, report.completedCycles,
                report.eligibilityQueriesPreserved, report.rejectedRequestsPreserved, report.motionCommandsAtAcknowledgement });
        }
        finally { c.TryExecute(CellRunCommand.Stop, out _); Time.timeScale = scale; }
    }

    public static async Task<object> History()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var fg = l.Fanuc.GetComponent<RobotGripper>();
        var hashes = Sources(); float scale = Time.timeScale;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1 || t.IsFaulted);
            var waiting = AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "REMOVE", .8f);
            Require(t.TryGetItem("P-000001", out var completed), "Initial product missing.");
            await Move(sg, completed, l.InputStop); Require(sg.TryGrip(completed, out var e), e);
            await Move(sg, completed, l.HandoffPoint); Require(sg.TryRelease(out e), e);
            var retreat = l.ScaraHandoffWaypoints.Retreat;
            Require(l.Scara.TryMoveTcp(retreat.PositionMetres, Yaw(retreat.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out e), e);
            await Until(() => sg.IsStopped);
            await Move(fg, completed, l.HandoffPoint); Require(fg.TryGrip(completed, out e), e);
            await Move(fg, completed, l.OutputA); Require(fg.TryRelease(out e), e);
            Require(t.TryRemove("REMOVE", ProductLocation.Waiting, out e), e);
            UnityEngine.Object.Destroy(completed.gameObject); await Ticks(2);
            waiting.Body.position = l.InputStop.position; waiting.Body.rotation = l.InputStop.rotation;
            Require(t.TryFeedNext(out var fed, out e) && fed.Id == "WAIT-A", e);
            AddWaiting(l, "TAIL-A", .6f); AddWaiting(l, "TAIL-B", .9f);
            await Move(sg, waiting, l.InputStop); Require(sg.TryGrip(waiting, out e), e);
            var q = sg.GetComponent<RobotWaypointSequence>(); Require(q.TryStart(out e), e); q.Pause(); Require(c.TryTakeControl(q, out e), e);
            Time.timeScale = .1f; Require(c.TryExecute(CellRunCommand.Start, out e), e); await Ticks(1);
            c.enabled = false; c.enabled = true; Time.timeScale = 1; await Ticks(2);
            string cause = c.FaultReason, before = Conservation(l), records = Products(l);
            Require(c.TryExecute(CellRunCommand.Reset, out e), e);
            Require(c.State == CellRunState.Stopped && Conservation(l) == before && c.LastAcknowledgedFaultReason == cause &&
                t.Count == 5 && !t.TryGetItem("P-000001", out _) && waiting.Owner == sg && sg.HeldItem == waiting &&
                t.WaitingSnapshot().SequenceEqual(new[] { "TAIL-A", "TAIL-B" }), "Acknowledgement erased history/holding/FIFO or recreated terminal object.");
            Time.timeScale = 5; Require(c.TryExecute(CellRunCommand.Start, out e), e);
            await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && Products(l) == records && waiting.Owner == sg, c.FaultReason);
            Require(c.TryExecute(CellRunCommand.Stop, out e) && c.TryReleaseControl(out e), e);
            var duplicate = UnityEngine.Object.Instantiate(l.ProductPrefab); duplicate.Body.position = l.InputStop.position + Vector3.right * 1.2f;
            Require(!t.TryRegister(duplicate, "P-000001", ProductDestination.A, out _) && !duplicate.IsTracked && Products(l) == records,
                "Completed ID became reusable after acknowledgement.");
            // A different later cause must not replace the prior acknowledgement until
            // this new fault is also explicitly acknowledged.
            l.Fanuc.enabled = false; await Ticks(2); l.Fanuc.enabled = true; await Ticks(2);
            string later = c.FaultReason;
            Require(c.IsFaulted && later != cause && c.LastAcknowledgedFaultReason == cause, "Later fault overwrote acknowledged history.");
            before = Conservation(l); Require(c.TryExecute(CellRunCommand.Reset, out e), e);
            Require(c.State == CellRunState.Stopped && c.LastAcknowledgedFaultReason == later && Conservation(l) == before, "Second acknowledgement lost history/data.");
            var snapshot = t.Snapshot();
            Require(snapshot.Count(r => r.Location == ProductLocation.OutputA) == 1 && snapshot.Count(r => r.Location == ProductLocation.Removed) == 1 &&
                snapshot.Count(r => r.Location == ProductLocation.Waiting) == 2 && snapshot.Count(r => r.Location == ProductLocation.Scara) == 1 && !t.IsFaulted,
                "Final conservation differs from 5 = completed 1 + removed 1 + confirmed residual 3.");
            CheckSources(hashes);
            var report = new { passed = true, verifiedUtc = DateTime.UtcNow, registeredProducts = 5, completedA = 1, explicitlyRemoved = 1,
                physicallyConfirmedResidual = 3, unconfirmedProducts = 0, explicitAcknowledgements = 2, retainedHeldId = waiting.ProductId,
                waitingOrder = t.WaitingSnapshot(), completedObjectRecreated = false, completedIdReuse = false, acknowledgedCauses = new[] { cause, later },
                motionCommandsAtAcknowledgement = 0, sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, hashes,
                scope = "API-prepared terminal/removal histories and tracked SCARA holding. Acknowledgement clears no data and issues no path/motion command." };
            Write("cell-fault-recovery-history.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.registeredProducts, report.completedA, report.explicitlyRemoved,
                report.physicallyConfirmedResidual, report.unconfirmedProducts, report.explicitAcknowledgements, report.completedIdReuse });
        }
        finally { c.TryExecute(CellRunCommand.Stop, out _); Time.timeScale = scale; }
    }

    public static async Task<object> Blocked(int scenario)
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var q = sg.GetComponent<RobotWaypointSequence>();
        var original = q.Waypoints; RobotWaypointSet copy = null;
        var hashes = Sources(); float scale = Time.timeScale; bool paused = EditorApplication.isPaused; int rejected = 0; bool repaired = false;
        float originalLink = 0;
        void Reject(string label)
        {
            string before = Full(l);
            Require(!c.TryExecute(CellRunCommand.Reset, out var e) && !string.IsNullOrWhiteSpace(e) && Full(l) == before,
                "Rejected acknowledgement changed fault/data/commands: " + label); rejected++;
        }
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1 || t.IsFaulted); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
            Require(t.TryGetItem("P-000001", out var item), "Initial product missing.");
            if (scenario != 2)
            {
                if (scenario == 3 || scenario == 5)
                { copy = UnityEngine.Object.Instantiate(original); SetWaypoints(q, copy); }
                await Move(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
                Require(q.TryStart(out e), e); q.Pause(); if (scenario != 5) Require(c.TryTakeControl(q, out e), e);
            }
            if (scenario == 5)
            {
                // The adopted movement-only case is covered by VerifyPathFaultRecovery.
                Time.timeScale = .1f; Require(q.TryResume(out var e), e);
                SetRetreat(copy, original.Retreat.PositionMetres + Vector3.right * .01f);
                await Until(() => q.State == State.Faulted && c.IsFaulted);
                SetRetreat(copy, original.Retreat.PositionMetres);
            }
            else { c.enabled = false; if (scenario != 6) c.enabled = true; }
            string cause = c.FaultReason;
            if (scenario == 0) { t.enabled = false; t.enabled = true; }
            else if (scenario == 1) { item.gameObject.SetActive(false); item.gameObject.SetActive(true); }
            else if (scenario == 2)
            {
                var position = item.Body.position; item.Body.position = position + Vector3.right * .01f;
                Require(!c.CanExecute(CellRunCommand.Reset, out _), "Station mismatch admitted."); Reject("current physical station mismatch");
                await Until(() => t.IsFaulted); item.Body.position = position;
            }
            else if (scenario == 3) SetRetreat(copy, original.Retreat.PositionMetres + Vector3.right * .01f);
            else if (scenario == 4)
            {
                var so = new SerializedObject(l.Scara); var field = so.FindProperty("fkLink1Metres"); originalLink = field.floatValue;
                field.floatValue = originalLink + .01f; so.ApplyModifiedPropertiesWithoutUndo();
            }
            else if (scenario != 5 && scenario != 6) throw new ArgumentException("Unknown rejection scenario.");
            if (scenario != 6) await Ticks(2);
            Require(c.IsFaulted && c.FaultReason == cause && c.LastAcknowledgedFaultReason == null, "Restoration auto-recovered or replaced first cause.");
            if (scenario != 0 && scenario != 2) Reject("unresolved scenario " + scenario);
            if (scenario == 0 || scenario == 2 || scenario == 3 || scenario == 4)
            {
                if (scenario == 3) SetRetreat(copy, original.Retreat.PositionMetres);
                else if (scenario == 4)
                {
                    var so = new SerializedObject(l.Scara); so.FindProperty("fkLink1Metres").floatValue = originalLink; so.ApplyModifiedPropertiesWithoutUndo();
                }
                Require(c.IsFaulted && c.FaultReason == cause, "Repair cleared fault automatically.");
                var trackerCause = t.FaultReason; var recordsBefore = JsonConvert.SerializeObject(t.Snapshot());
                if (scenario == 0 || scenario == 2) Require(t.IsFaulted && c.CanExecute(CellRunCommand.Reset, out _), "Restored physical consistency did not enable explicit recovery.");
                Require(c.TryExecute(CellRunCommand.Reset, out var e), e); repaired = true;
                Require(c.State == CellRunState.Stopped && c.LastAcknowledgedFaultReason == cause && recordsBefore == JsonConvert.SerializeObject(t.Snapshot()) &&
                    (scenario == 2 ? q.State == State.Idle && item.Owner == null : q.State == State.Paused && item.Owner == sg), "Explicit post-repair acknowledgement changed progress/holding.");
                if (scenario == 0 || scenario == 2) Require(!t.IsFaulted && t.LastAcknowledgedFaultReason == trackerCause, "Tracker cause was not acknowledged/preserved.");
            }
            else if (scenario == 6)
            {
                c.enabled = true; Require(c.IsFaulted, "Re-enable acknowledged fault.");
                Time.timeScale = 0; Reject("zero-time acknowledgement");
                Time.timeScale = 1; EditorApplication.isPaused = true; Reject("editor-paused acknowledgement"); EditorApplication.isPaused = false;
                Require(c.TryExecute(CellRunCommand.Reset, out var e), e); repaired = true;
                Require(c.State == CellRunState.Stopped && q.State == State.Paused && item.Owner == sg, "Clock restoration auto-restarted program.");
            }
            else Require(c.IsFaulted && c.FaultReason == cause && !c.CanExecute(CellRunCommand.Reset, out _), "Deferred tracker/path fault was acknowledged.");
            Require(t.Count == 3 && t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B" }) &&
                (scenario == 1 ? item.Owner == null && t.ScaraProductId == item.ProductId : scenario == 2 ? item.Owner == null && t.InputProductId == item.ProductId : item.Owner == sg),
                "Rejected/explicit acknowledgement lost registry/FIFO/owner evidence.");
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, rejectedRequestsPreserved = rejected,
                repairedThenExplicitlyAcknowledged = repaired, automaticRecovery = false, registeredProducts = 3,
                unconfirmedProducts = scenario == 1 ? 1 : 0, lastRecordsRetained = true, sourcesPreserved = hashes.Count,
                directButtonClicks = 0, callbacksInjected = 0, hashes };
            Write("cell-fault-recovery-blocked-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.rejectedRequestsPreserved,
                report.repairedThenExplicitlyAcknowledged, report.automaticRecovery, report.unconfirmedProducts });
        }
        finally
        {
            EditorApplication.isPaused = paused; Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _);
            if (copy != null) { SetWaypoints(q, original); UnityEngine.Object.Destroy(copy); }
        }
    }

    private static void SetWaypoints(RobotWaypointSequence q, RobotWaypointSet set)
    { var so = new SerializedObject(q); so.FindProperty("waypoints").objectReferenceValue = set; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetRetreat(RobotWaypointSet set, Vector3 position)
    { var so = new SerializedObject(set); so.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = position; so.ApplyModifiedPropertiesWithoutUndo(); }

    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh unpaused LineWatch Play Mode.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch."); return l;
    }
    private static GrippableItem AddWaiting(LineWatchCellLayout l, string id, float offset)
    {
        var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset;
        Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e); return item;
    }
    private static async Task Move(RobotGripper g, GrippableItem item, Transform station)
    {
        var point = item.GetGripPoint(g.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc);
        var rotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = station.TransformPoint(point.localPosition) - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        var localRotation = Quaternion.Inverse(g.RobotFrame.rotation) * rotation; string error;
        bool ok = g.Scara != null ? g.Scara.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), Yaw(localRotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out error)
            : g.Fanuc.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), localRotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error);
        Require(ok, error); await Until(() => g.IsStopped); await Ticks(2);
    }
    private static string Products(LineWatchCellLayout l)
    {
        var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { t.IsFaulted, t.FaultReason, records = t.Snapshot(), fifo = t.WaitingSnapshot(),
            slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            objects = t.Snapshot().Select(r => new { r.Id, instance = t.TryGetItem(r.Id, out var item) ? item.GetInstanceID() : 0,
                owner = item != null && item.Owner != null ? item.Owner.GetInstanceID() : 0 }) });
    }
    private static string Conservation(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); return JsonConvert.SerializeObject(new { products = Products(l), c.HasControlledSequence,
            controlled = c.ControlledSequence != null ? c.ControlledSequence.GetInstanceID() : 0,
            robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() }.Select(g => {
                var q = g.GetComponent<RobotWaypointSequence>(); var held = g.HeldItem;
                return new { version = g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion,
                    current = g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition } : g.Fanuc.Joints.Select(a => a.CurrentPosition).ToArray(),
                    targets = g.Scara != null ? new[] { g.Scara.Axis1.TargetPosition, g.Scara.Axis2.TargetPosition, g.Scara.Axis3.TargetPosition, g.Scara.Axis4.TargetPosition } : g.Fanuc.Joints.Select(a => a.TargetPosition).ToArray(),
                    q.State, q.StepIndex, q.GripCount, q.ReleaseCount, q.IsPickPlace, held = held != null ? held.GetInstanceID() : 0,
                    position = held != null ? V(held.Body.position) : null, rotation = held != null ? Q(held.Body.rotation) : null };
            }) });
    }
    private static string Full(LineWatchCellLayout l)
    { var c = l.GetComponent<CellRunController>(); return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.LastAcknowledgedFaultReason, c.enabled, conserved = Conservation(l) }); }
    private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }
        .SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))).Replace("-", ""); }
    private static void CheckSources(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification changed sources/assets.");
    private static async Task Ticks(int n) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= n * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition)
    { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string name, object data) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + name, JsonConvert.SerializeObject(data, Formatting.Indented)); }
    private static void Require(bool ok, string error) { if (!ok) throw new InvalidOperationException(error); }
}
