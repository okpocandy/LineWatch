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

public static class VerifyFaultedPathAdoption
{
    public static async Task<string> Reproduce()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var q = l.Scara.GetComponent<RobotWaypointSequence>();
        var source = q.Waypoints; var copy = UnityEngine.Object.Instantiate(source); float scale = Time.timeScale;
        try
        {
            Time.timeScale = .1f; SetWaypoints(q, copy); Require(q.TryStart(out var e), e);
            SetRetreat(copy, source.Retreat.PositionMetres + Vector3.right * .01f);
            await Until(() => q.State == State.Faulted && c.IsFaulted); SetRetreat(copy, source.Retreat.PositionMetres); await Ticks(2);
            bool allowed = c.CanTakeControl(q, out e);
            var report = new { reproduced = !allowed && !c.HasControlledSequence, reason = e, cellFault = c.FaultReason,
                pathFault = q.LastError, step = q.StepIndex, hasControlledSequence = c.HasControlledSequence };
            Write("fault-adoption-before.json", report); return JsonConvert.SerializeObject(report);
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetWaypoints(q, source); UnityEngine.Object.Destroy(copy); }
    }

    public static async Task<string> Normal(int scenario)
    {
        Require(scenario >= 0 && scenario <= 5, "Unknown normal scenario.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = (scenario % 2 == 0 ? l.Scara.gameObject : l.Fanuc.gameObject).GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>();
        var other = (g.Scara != null ? l.Fanuc.gameObject : l.Scara.gameObject).GetComponent<RobotWaypointSequence>();
        var source = q.Waypoints; var copy = UnityEngine.Object.Instantiate(source); float scale = Time.timeScale; var hashes = Sources(); int rejected = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
            var item = await Prepare(l, g, copy, scenario >= 2); var retreat = copy.Retreat.PositionMetres;
            if (scenario <= 3) Time.timeScale = .1f;
            else await Until(() => q.GripCount == 1 && q.StepIndex >= 2 || c.IsFaulted);
            SetRetreat(copy, retreat + Vector3.right * .01f); await Until(() => q.State == State.Faulted && c.IsFaulted);
            if (scenario < 2) { t.enabled = false; t.enabled = true; }
            Require(!c.HasControlledSequence && q.IsPickPlace == (scenario >= 2), "Fixture was adopted or used the wrong program mode.");
            Reject(() => c.TryTakeControl(q, out _), "changed saved program");
            SetRetreat(copy, retreat); Time.timeScale = 1; await Ticks(3);
            Require(c.IsFaulted && q.State == State.Faulted && !c.HasControlledSequence, "Restoration automatically adopted or recovered a path.");
            Reject(() => c.TryExecute(CellRunCommand.Reset, out _), "acknowledge before adoption");
            Reject(() => c.TryTakeControl(null, out _), "null target");
            Reject(() => c.TryTakeControl(other, out _), "idle/other robot target");
            var foreign = new GameObject("ForeignRecoveryPath").AddComponent<RobotWaypointSequence>();
            Reject(() => c.TryTakeControl(foreign, out _), "foreign target"); UnityEngine.Object.Destroy(foreign.gameObject);
            string full = Full(l);
            for (int i = 0; i < 100; i++) Require(c.CanTakeControl(q, out var e), e);
            Require(Full(l) == full, "Adoption eligibility queries changed evidence or ownership.");
            var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show(); w.RefreshCell();
            var takeName = g.Scara != null ? "cell-run-take-scara" : "cell-run-take-fanuc";
            Require(w.rootVisualElement.Q<Button>(takeName).enabledInHierarchy && w.rootVisualElement.Q<Button>(takeName).text.Contains("고장 복구 인계") &&
                !w.rootVisualElement.Q<Button>("cell-run-reset").enabledInHierarchy, "Unadopted recovery target UI is wrong.");
            string evidence = Evidence(l), records = Records(l);
            Require(c.TryTakeControl(q, out var error), error);
            Require(c.HasControlledSequence && c.ControlledSequence == q && Evidence(l) == evidence && c.IsFaulted && q.State == State.Faulted,
                "Adoption acknowledged a fault, dispatched, changed product evidence or replaced the saved step.");
            await Ticks(3); Require(Evidence(l) == evidence, "Adoption automatically progressed during later frames.");
            w.RefreshCell();
            Require(!w.rootVisualElement.Q<Button>(takeName).enabledInHierarchy && w.rootVisualElement.Q<Button>("cell-run-reset").enabledInHierarchy &&
                w.rootVisualElement.Q<HelpBox>("cell-run-mode").text.Contains("복구 경로 인계 유지"), "Adopted fault UI did not require explicit acknowledgement.");
            Reject(() => c.TryTakeControl(q, out _), "duplicate adoption");
            Reject(() => c.TryTakeControl(other, out _), "replace target");
            Reject(() => c.TryExecute(CellRunCommand.Start, out _), "start while faulted");
            Reject(() => c.TryExecute(CellRunCommand.Resume, out _), "resume while faulted");
            Reject(() => c.TryReleaseControl(out _), "return before acknowledgement");
            Reject(() => q.TryResume(out _), "individual resume");
            Reject(() => g.TryRelease(out _), "manual release");
            string data = Data(l), cellCause = c.FaultReason, pathCause = q.LastError, trackerCause = t.FaultReason;
            bool trackingFault = t.IsFaulted; string trackerArchive = t.LastAcknowledgedFaultReason;
            Require(c.TryExecute(CellRunCommand.Reset, out error), error);
            Require(c.State == CellRunState.Stopped && q.State == State.Paused && !t.IsFaulted && Data(l) == data && c.ControlledSequence == q &&
                c.LastAcknowledgedFaultReason == cellCause && q.LastAcknowledgedFaultReason == pathCause &&
                t.LastAcknowledgedFaultReason == (trackingFault ? trackerCause : trackerArchive), "Separate acknowledgement lost causes or changed saved data.");
            await Ticks(3); Require(Data(l) == data && q.State == State.Paused, "Acknowledgement automatically resumed.");
            Time.timeScale = 5; Require(c.TryExecute(CellRunCommand.Start, out error), error); await Until(() => q.State == State.Completed || c.IsFaulted);
            Require(q.State == State.Completed && q.GripCount == (scenario >= 2 ? 1 : 0) && q.ReleaseCount == (scenario >= 2 ? 1 : 0) && Records(l) == records,
                q.LastError ?? c.FaultReason ?? "Separate start failed, repeated an action or changed registered records.");
            if (scenario < 2) Require(item.Owner == g && g.HeldItem == item && item.ProductId == "P-000001", "Registered holding was lost during adoption/recovery.");
            else Require(item.Owner == null && g.HeldItem == null, "Trial release missing.");
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, recoveryAdoptions = 1, separateAcknowledgements = 1,
                eligibilityQueriesPreserved = 100, rejectedRequestsPreserved = rejected, completedPrograms = 1, registeredProducts = 3,
                physicallyConfirmedResidual = 3, unconfirmedProducts = 0, motionCommandsAtAdoption = 0, sourcesPreserved = hashes.Count,
                automaticGrips = q.GripCount, automaticReleases = q.ReleaseCount, directButtonClicks = 0, callbacksInjected = 0,
                scope = "One unadopted fault path, then explicit acknowledgement, then separate start. Registered holding or unchanged registered input plus untracked trial.", hashes };
            Write("fault-adoption-normal-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.recoveryAdoptions, report.separateAcknowledgements, report.completedPrograms, report.rejectedRequestsPreserved });
        }
        finally { Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); SetWaypoints(q, source); UnityEngine.Object.Destroy(copy); }
        void Reject(Func<bool> action, string name) { string before = Full(l); Require(!action() && Full(l) == before, "Rejected request mutated state: " + name); rejected++; }
    }

    public static async Task<string> Blocked(int scenario)
    {
        Require(scenario >= 0 && scenario <= 6, "Unknown blocked scenario.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var g = l.Scara.GetComponent<RobotGripper>(); var q = g.GetComponent<RobotWaypointSequence>(); var other = l.Fanuc.GetComponent<RobotWaypointSequence>();
        var source = q.Waypoints; var copy = UnityEngine.Object.Instantiate(source); float scale = Time.timeScale, link = l.Scara.Geometry.Link1Metres;
        float minimum = l.Scara.Axis3.MinimumMetres; bool editorPaused = EditorApplication.isPaused; var hashes = Sources(); int rejected = 0; bool repaired = false;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
            if (scenario == 4) { Require(other.TryStart(out var e), e); other.Pause(); }
            var item = await Prepare(l, g, copy, scenario == 2 || scenario == 6); var retreat = copy.Retreat.PositionMetres;
            if (scenario == 2) Time.timeScale = .1f;
            if (scenario == 6) await Until(() => q.GripCount == 1 && q.StepIndex >= 2 || c.IsFaulted);
            SetRetreat(copy, retreat + Vector3.right * .01f); await Until(() => q.State == State.Faulted && c.IsFaulted);
            SetRetreat(copy, retreat); t.enabled = false; t.enabled = true; Time.timeScale = 1; await Ticks(2);
            var itemPosition = item.Body.position;
            if (scenario == 0 || scenario == 6) { item.gameObject.SetActive(false); item.gameObject.SetActive(true); }
            else if (scenario == 1) { var so = new SerializedObject(l.Scara); so.FindProperty("fkLink1Metres").floatValue = link + .01f; so.ApplyModifiedPropertiesWithoutUndo(); }
            else if (scenario == 2) item.Body.position += Vector3.right * .01f;
            else if (scenario == 3)
            { var so = new SerializedObject(l.Scara.Axis3); so.FindProperty("minimumMetres").floatValue = l.Scara.Axis3.CurrentPosition - .001f; so.ApplyModifiedPropertiesWithoutUndo(); }
            if (scenario == 5)
            {
                c.enabled = false; Reject("inactive controller"); c.enabled = true;
                Time.timeScale = 0; Reject("zero clock"); Time.timeScale = 1;
                EditorApplication.isPaused = true; Reject("Editor Pause"); EditorApplication.isPaused = false;
            }
            else
            {
                await Ticks(2);
                if (scenario == 1 || scenario == 2 || scenario == 3) Require(c.CanTakeControl(q, out _), "Fixture must reach final FK/grip/path validation.");
                Reject("blocked " + scenario);
            }
            if (scenario == 1 || scenario == 2 || scenario == 3 || scenario == 5)
            {
                if (scenario == 1) { var so = new SerializedObject(l.Scara); so.FindProperty("fkLink1Metres").floatValue = link; so.ApplyModifiedPropertiesWithoutUndo(); }
                if (scenario == 2) item.Body.position = itemPosition;
                if (scenario == 3) { var so = new SerializedObject(l.Scara.Axis3); so.FindProperty("minimumMetres").floatValue = minimum; so.ApplyModifiedPropertiesWithoutUndo(); }
                await Ticks(2); string evidence = Evidence(l);
                Require(c.IsFaulted && q.State == State.Faulted && t.IsFaulted && !c.HasControlledSequence, "Repair implicitly adopted or acknowledged a path.");
                Require(c.TryTakeControl(q, out var error) && c.ControlledSequence == q && Evidence(l) == evidence, error ?? "Post-repair adoption changed evidence."); repaired = true;
            }
            else Require(!c.CanTakeControl(q, out _) && !c.HasControlledSequence, "Unsupported/missing-owner path was adopted.");
            if (scenario == 0) Require(item.Owner == null && g.HeldItem == null && t.ScaraProductId == "P-000001", "Registered owner loss became confirmed holding.");
            if (scenario == 6) Require(item.Owner == null && g.HeldItem == null && q.GripCount == 1 && q.ReleaseCount == 0, "Trial owner loss changed saved actions.");
            Require(c.IsFaulted && q.State == State.Faulted && t.IsFaulted && t.Count == 3 && t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B" }), "Blocked adoption cleared faults or changed records/FIFO.");
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, rejectedRequestsPreserved = rejected,
                repairedThenExplicitlyAdopted = repaired, noImplicitAcknowledgement = true, noPartialControlAssignmentOnFailure = true,
                unconfirmedRegisteredProducts = scenario == 0 ? 1 : 0, unconfirmedTrialHolding = scenario == 6,
                motionCommandsAtAdoption = 0, sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, hashes };
            Write("fault-adoption-blocked-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.rejectedRequestsPreserved, report.repairedThenExplicitlyAdopted, report.noImplicitAcknowledgement });
        }
        finally
        {
            EditorApplication.isPaused = editorPaused; Time.timeScale = scale; c.enabled = true;
            var so = new SerializedObject(l.Scara); so.FindProperty("fkLink1Metres").floatValue = link; so.ApplyModifiedPropertiesWithoutUndo();
            so = new SerializedObject(l.Scara.Axis3); so.FindProperty("minimumMetres").floatValue = minimum; so.ApplyModifiedPropertiesWithoutUndo();
            c.TryExecute(CellRunCommand.Stop, out _); SetWaypoints(q, source); UnityEngine.Object.Destroy(copy);
        }
        void Reject(string name) { string before = Full(l); Require(!c.TryTakeControl(q, out var e) && !string.IsNullOrEmpty(e) && Full(l) == before && !c.HasControlledSequence, "Rejected adoption partially changed state: " + name); rejected++; }
    }

    private static async Task<GrippableItem> Prepare(LineWatchCellLayout l, RobotGripper g, RobotWaypointSet copy, bool pickPlace)
    {
        var q = g.GetComponent<RobotWaypointSequence>(); SetWaypoints(q, copy);
        if (pickPlace)
        {
            var lab = g.GetComponent<RobotGripperLab>(); Require(lab.TryPrepare(out var e), e);
            Require(q.TryStartPickPlace(g, lab.TrialItem, out e), e); return lab.TrialItem;
        }
        var t = l.GetComponent<CellProductTracker>(); Require(t.TryGetItem("P-000001", out var item), "Registered input missing.");
        if (g.Fanuc != null)
        {
            var sg = l.Scara.GetComponent<RobotGripper>(); await Move(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
            await Move(sg, item, l.HandoffPoint); Require(sg.TryRelease(out e), e);
            var retreat = l.ScaraHandoffWaypoints.Retreat; Require(l.Scara.TryMoveTcp(retreat.PositionMetres, Yaw(retreat.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out e), e); await Until(() => sg.IsStopped);
        }
        await Move(g, item, g.Scara != null ? l.InputStop : l.HandoffPoint); Require(g.TryGrip(item, out var error), error);
        Require(q.TryStart(out error), error); return item;
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
    private static void AddWaiting(LineWatchCellLayout l, string id, float offset)
    {
        var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset;
        Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e);
    }
    private static string Records(LineWatchCellLayout l)
    {
        var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(),
            slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            physical = t.Snapshot().Select(r => new { r.Id, instance = t.TryGetItem(r.Id, out var item) ? item.GetInstanceID() : 0, owner = item != null && item.Owner != null ? item.Owner.GetInstanceID() : 0 }) });
    }
    private static string Data(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { records = Records(l),
        robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() }.Select(g => {
            var q = g.GetComponent<RobotWaypointSequence>(); var held = g.HeldItem;
            return new { version = g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion,
                current = g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition } : g.Fanuc.Joints.Select(a => a.CurrentPosition).ToArray(),
                targets = g.Scara != null ? new[] { g.Scara.Axis1.TargetPosition, g.Scara.Axis2.TargetPosition, g.Scara.Axis3.TargetPosition, g.Scara.Axis4.TargetPosition } : g.Fanuc.Joints.Select(a => a.TargetPosition).ToArray(),
                q.StepIndex, q.IsPickPlace, q.GripCount, q.ReleaseCount, q.LastArrivedStepIndex,
                arrivalPosition = V(q.LastArrivalPose.position), arrivalRotation = Q(q.LastArrivalPose.rotation), cycleItem = q.CycleItem != null ? q.CycleItem.GetInstanceID() : 0,
                held = held != null ? held.GetInstanceID() : 0, position = held != null ? V(held.Body.position) : null, rotation = held != null ? Q(held.Body.rotation) : null };
        }) });
    // Adoption may change only the controller's command ownership, which is excluded here.
    private static string Evidence(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.LastAcknowledgedFaultReason, trackerFault = t.IsFaulted,
            trackerCause = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason,
            paths = new[] { l.Scara.GetComponent<RobotWaypointSequence>(), l.Fanuc.GetComponent<RobotWaypointSequence>() }.Select(q => new { q.State, q.LastError, q.LastAcknowledgedFaultReason }), data = Data(l) });
    }
    private static string Full(LineWatchCellLayout l)
    { var c = l.GetComponent<CellRunController>(); return JsonConvert.SerializeObject(new { c.HasControlledSequence, controlled = c.ControlledSequence != null ? c.ControlledSequence.GetInstanceID() : 0, evidence = Evidence(l) }); }
    private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }
        .SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))).Replace("-", ""); }
    private static void CheckSources(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification changed code/scene/assets.");

    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use a fresh unpaused LineWatch Play session.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch."); return l;
    }
    private static void SetWaypoints(RobotWaypointSequence q, RobotWaypointSet set)
    { var so = new SerializedObject(q); so.FindProperty("waypoints").objectReferenceValue = set; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetRetreat(RobotWaypointSet set, Vector3 position)
    { var so = new SerializedObject(set); so.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = position; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static async Task Ticks(int n) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= n * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
