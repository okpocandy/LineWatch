using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using State = LineWatch.RobotWaypointSequence.RunState;

public static class VerifyCellRunHandoff
{
    public static async Task<object> Play()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() };
        var sequences = robots.Select(g => g.GetComponent<RobotWaypointSequence>()).ToArray();
        var hashes = Sources(); var rows = new List<object>(); var rejected = new List<string>();
        var originals = sequences.Select(q => q.Waypoints).ToArray();
        var copies = originals.Select(UnityEngine.Object.Instantiate).ToArray();
        float scale = Time.timeScale, maxPosition = 0, maxAngle = 0;
        int adoptions = 0, releases = 0, completions = 0, pausedChecks = 0, externalStops = 0;
        void Reject(Func<bool> request, string label)
        { string before = Signature(l); Require(!request() && Signature(l) == before, "Rejected command mutated state: " + label); rejected.Add(label); }
        void Command(CellRunCommand command)
        { Require(c.TryExecute(command, out var e), command + ": " + e); }
        try
        {
            await Until(() => t.Count == 1 || t.IsFaulted); Require(!c.IsFaulted && !t.IsFaulted, c.FaultReason ?? t.FaultReason);
            AddWaiting(l, t); string products = Products(l);
            for (int r = 0; r < 2; r++) SetWaypoints(sequences[r], copies[r]);
            Reject(() => c.TryTakeControl(null, out _), "null program");
            Reject(() => c.TryTakeControl(sequences[0], out _), "idle program");
            Reject(() => c.TryReleaseControl(out _), "no owned program");
            var foreign = new GameObject("ForeignSequence").AddComponent<RobotWaypointSequence>();
            Reject(() => c.TryTakeControl(foreign, out _), "foreign program");
            UnityEngine.Object.Destroy(foreign.gameObject);
            for (int r = 0; r < 2; r++)
            for (int mode = 0; mode < 2; mode++)
            {
                var g = robots[r]; var q = sequences[r]; var other = sequences[1 - r]; var lab = g.GetComponent<RobotGripperLab>();
                Time.timeScale = 5;
                Require(lab.TryPrepare(out var e) && lab.TryStartPickPlace(out e), e);
                if (mode == 1) await Until(() => q.GripCount == 1 && q.StepIndex == 2 || q.State == State.Faulted);
                Require(q.IsBusy, q.LastError ?? "Program completed before handoff fixture.");
                Reject(() => c.TryTakeControl(q, out _), "moving handoff");
                q.Pause(); int step = q.StepIndex, grips = q.GripCount, drops = q.ReleaseCount;
                var retreat = copies[r].Retreat.PositionMetres;
                SetRetreat(copies[r], retreat + Vector3.right * .01f);
                Reject(() => c.TryTakeControl(q, out _), "changed program before handoff");
                SetRetreat(copies[r], retreat);
                string robotBefore = RobotSignature(l); var bodyBefore = lab.TrialItem.Body.position;
                Require(c.TryTakeControl(q, out e), e); adoptions++;
                Require(c.State == CellRunState.Stopped && c.ControlledSequence == q && c.HasControlledSequence && RobotSignature(l) == robotBefore,
                    "Adoption moved or reset the robot.");
                Reject(() => c.TryTakeControl(q, out _), "duplicate handoff");
                Reject(() => q.TryResume(out _), "individual resume while owned/stopped");
                Reject(() => other.TryStart(out _), "other start while owned/stopped");
                Reject(() => g.TryRelease(out _), "manual release while owned/stopped");
                SetRetreat(copies[r], retreat + Vector3.right * .01f);
                Reject(() => c.TryExecute(CellRunCommand.Start, out _), "invalid common start preflight");
                SetRetreat(copies[r], retreat);
                Time.timeScale = .1f;
                uint version = Version(g); Command(CellRunCommand.Start);
                Require(c.State == CellRunState.Running && q.IsBusy && Version(g) > version && q.StepIndex == step && q.GripCount == grips && q.ReleaseCount == drops,
                    "Common Start did not resume the saved stage.");
                Reject(() => c.TryExecute(CellRunCommand.Start, out _), "duplicate common start");
                Reject(() => c.TryReleaseControl(out _), "release while running");
                Reject(() => q.TryStart(out _), "individual start while owned/running");
                Reject(() => q.TryResume(out _), "individual resume while owned/running");
                Reject(() => other.TryStart(out _), "other start while owned/running");
                Reject(() => l.Scara.MoveJoint(0, l.Scara.Axis1.CurrentPosition + 1), "manual SCARA during common run");
                Reject(() => l.Fanuc.Joints[0].MoveTo(l.Fanuc.Joints[0].CurrentPosition + 1), "raw FANUC during common run");
                Reject(() => g.TryRelease(out _), "manual release during common run");
                Reject(() => t.TryRemove("WAIT-1", ProductLocation.Waiting, out _), "remove during common run");
                await Ticks(1);
                Require(q.IsBusy, "Movement finished before common pause.");
                Command(CellRunCommand.Pause);
                Time.timeScale = 1;
                var stopped = RobotSignature(l); var body = lab.TrialItem.Body.position;
                await Ticks(3);
                Require(c.State == CellRunState.Paused && RobotSignature(l) == stopped && q.StepIndex == step &&
                    Vector3.Distance(lab.TrialItem.Body.position, body) < .00001f && q.GripCount == grips && q.ReleaseCount == drops &&
                    lab.TrialItem.Owner == (mode == 1 ? g : null), "Common Pause lost pose/stage/holding."); pausedChecks++;
                Reject(() => c.TryReleaseControl(out _), "release while paused");
                Reject(() => c.TryExecute(CellRunCommand.Start, out _), "start while paused");
                Reject(() => q.TryResume(out _), "individual resume while owned/paused");
                SetRetreat(copies[r], retreat + Vector3.right * .01f);
                Reject(() => c.TryExecute(CellRunCommand.Resume, out _), "changed program common resume");
                SetRetreat(copies[r], retreat);
                if (mode == 0)
                {
                    lab.TrialItem.Body.position = bodyBefore + Vector3.right * .01f;
                    Reject(() => c.TryExecute(CellRunCommand.Resume, out _), "misaligned pre-grip common resume");
                    lab.TrialItem.Body.position = bodyBefore;
                }
                Time.timeScale = 0;
                Reject(() => c.TryExecute(CellRunCommand.Resume, out _), "zero-time common resume");
                Command(CellRunCommand.Stop); // Stop remains possible even when time is frozen.
                Require(c.State == CellRunState.Stopped && c.ControlledSequence == q && q.StepIndex == step, "Stop discarded command ownership/stage.");
                Reject(() => c.TryExecute(CellRunCommand.Resume, out _), "resume from stopped");
                Reject(() => q.TryResume(out _), "individual resume after common stop");
                Time.timeScale = .1f;
                Require(c.TryReleaseControl(out e), e); releases++;
                Require(!c.HasControlledSequence && q.StepIndex == step && q.GripCount == grips && q.ReleaseCount == drops, "Release reset the saved program.");
                bool accepted = g.Scara != null ? g.Scara.MoveJoint(0, g.Scara.Axis1.CurrentPosition + 1) : g.Fanuc.MoveJoint(0, g.Fanuc.Joints[0].CurrentPosition + 1);
                Require(accepted, "Explicit release did not restore manual permission."); g.Stop();
                Require(c.TryTakeControl(q, out e), e); adoptions++;
                Command(CellRunCommand.Start);
                // An individual stop interrupts the cell too, without an automatic retry.
                g.Stop();
                await Until(() => c.State == CellRunState.Paused || c.IsFaulted);
                Require(c.State == CellRunState.Paused && q.State == State.Paused && q.StepIndex == step && !c.IsFaulted, c.FaultReason);
                externalStops++;
                Time.timeScale = 5; Command(CellRunCommand.Resume);
                await Until(() => q.State == State.Completed || q.State == State.Faulted || c.IsFaulted);
                Require(q.State == State.Completed && !c.IsFaulted && q.GripCount == 1 && q.ReleaseCount == 1 && g.HeldItem == null && lab.TrialItem.Owner == null,
                    q.LastError ?? c.FaultReason ?? "Actions repeated/skipped after common resume.");
                Require(lab.TrialItem.TryGetGripPose(g.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc, out var actual), "Grip pose missing.");
                var rotation = g.RobotFrame.rotation * q.Waypoints.Retreat.Rotation;
                var point = g.RobotFrame.TransformPoint(q.Waypoints.Retreat.PositionMetres) + rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
                float pe = Vector3.Distance(actual.position, point);
                float ae = RobotGripper.RotationError(actual.rotation, rotation * (Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation));
                Require(pe <= .002f && ae <= .5f, "Common program placed the trial part incorrectly.");
                maxPosition = Mathf.Max(maxPosition, pe); maxAngle = Mathf.Max(maxAngle, ae); completions++;
                Command(CellRunCommand.Pause); version = Version(g); Command(CellRunCommand.Resume);
                Require(Version(g) == version && q.State == State.Completed && q.GripCount == 1 && q.ReleaseCount == 1, "Completed path replayed on Resume.");
                Command(CellRunCommand.Stop); version = Version(g); Command(CellRunCommand.Start);
                Require(Version(g) == version && q.State == State.Completed, "Completed path replayed on Start.");
                Command(CellRunCommand.Stop);
                Require(c.TryReleaseControl(out e), e); releases++;
                Reject(() => c.TryReleaseControl(out _), "duplicate release");
                Require(Products(l) == products && !t.IsFaulted, "Untracked trial changed registered inventory/FIFO.");
                rows.Add(new { robot = r, mode = mode == 0 ? "before grip" : "holding", savedStep = step, q.GripCount, q.ReleaseCount, pe, ae });
            }
            CheckSources(hashes);
            var report = new { passed = true, verifiedUtc = DateTime.UtcNow, completions, adoptions, releases, pausedChecks, externalStops,
                rejectedRequestsPreserved = rejected.Count, registeredProducts = t.Count, physicallyConfirmedResidual = 3, unconfirmedProducts = 0,
                maxPositionErrorMetres = maxPosition, maxRotationErrorDegrees = maxAngle, sourcesPreserved = hashes.Count,
                directButtonClicks = 0, callbacksInjected = 0, rows, rejected, hashes,
                scope = "One explicitly adopted paused program at a time; trials are unregistered. Not a two-robot automatic product cycle." };
            Write("cell-run-handoff-play.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.completions, report.adoptions, report.releases, report.pausedChecks, report.externalStops,
                report.rejectedRequestsPreserved, report.maxPositionErrorMetres, report.maxRotationErrorDegrees, report.sourcesPreserved });
        }
        finally
        {
            c.TryExecute(CellRunCommand.Stop, out _); Time.timeScale = scale;
            for (int r = 0; r < 2; r++) { SetWaypoints(sequences[r], originals[r]); UnityEngine.Object.Destroy(copies[r]); }
        }
    }

    public static async Task<object> Registered()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var fg = l.Fanuc.GetComponent<RobotGripper>();
        var hashes = Sources(); float scale = Time.timeScale; int completions = 0, preserved = 0;
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1 || t.IsFaulted); AddWaiting(l, t);
            Require(t.TryGetItem("P-000001", out var item), "Initial tracked product missing.");
            foreach (var g in new[] { sg, fg })
            {
                var station = g == sg ? l.InputStop : l.HandoffPoint;
                await Move(g, item, station); Require(g.TryGrip(item, out var e), e);
                var q = g.GetComponent<RobotWaypointSequence>(); Require(q.TryStart(out e), e); q.Pause();
                Require(c.TryTakeControl(q, out e), e);
                string products = Products(l); int step = q.StepIndex;
                foreach (var command in new[] { CellRunCommand.Start, CellRunCommand.Pause, CellRunCommand.Resume, CellRunCommand.Stop, CellRunCommand.Start })
                {
                    Require(c.TryExecute(command, out e), e);
                    Require(Products(l) == products && q.StepIndex == step && item.Owner == g && g.HeldItem == item && q.GripCount == 0 && q.ReleaseCount == 0,
                        "Common command lost registered holding/ID/slot/FIFO."); preserved++;
                }
                await Until(() => q.State == State.Completed || c.IsFaulted || q.State == State.Faulted);
                Require(q.State == State.Completed && !c.IsFaulted && Products(l) == products && item.Owner == g && g.HeldItem == item, q.LastError ?? c.FaultReason);
                completions++;
                Require(c.TryExecute(CellRunCommand.Stop, out e) && c.TryReleaseControl(out e), e);
                if (g == sg)
                {
                    await Move(sg, item, l.HandoffPoint); Require(sg.TryRelease(out e), e);
                    var retreat = l.ScaraHandoffWaypoints.Retreat;
                    Require(l.Scara.TryMoveTcp(retreat.PositionMetres, Yaw(retreat.Rotation), ScaraJointLab.IkSelection.PositiveElbow, out _, out e), e);
                    await Until(() => sg.IsStopped);
                }
            }
            Require(t.Count == 3 && t.FanucProductId == item.ProductId && t.ScaraProductId == null && t.InputProductId == null && t.HandoffProductId == null &&
                t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-1", "WAIT-2" }) && !t.IsFaulted && !c.IsFaulted, "Final tracked accounting mismatch.");
            CheckSources(hashes);
            var report = new { passed = true, verifiedUtc = DateTime.UtcNow, completions, registeredHoldingChecks = preserved,
                registeredProducts = 3, physicallyConfirmedResidual = 3, unconfirmedProducts = 0, retainedProductId = item.ProductId,
                waitingOrder = t.WaitingSnapshot(), physicalOwner = "FANUC", sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, hashes,
                scope = "API-prepared tracked holding, one movement-only program per robot. Manual station transfer; no automatic two-robot product cycle." };
            Write("cell-run-handoff-registered.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.completions, report.registeredHoldingChecks, report.physicallyConfirmedResidual, report.unconfirmedProducts });
        }
        finally { c.TryExecute(CellRunCommand.Stop, out _); Time.timeScale = scale; }
    }

    public static async Task<object> Faults(int scenario)
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var fg = l.Fanuc.GetComponent<RobotGripper>();
        var q = sg.GetComponent<RobotWaypointSequence>(); float scale = Time.timeScale; var hashes = Sources();
        try
        {
            Time.timeScale = 5; await Until(() => t.Count == 1 || t.IsFaulted); AddWaiting(l, t);
            Require(t.TryGetItem("P-000001", out var item), "Initial sample missing.");
            await Move(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
            Require(q.TryStart(out e), e); q.Pause(); Require(c.TryTakeControl(q, out e), e);
            Time.timeScale = .1f; Require(c.TryExecute(CellRunCommand.Start, out e), e); await Ticks(1);
            Require(q.IsBusy && !sg.IsStopped, "Active controlled holding required.");
            int step = q.StepIndex; string records = JsonConvert.SerializeObject(t.Snapshot());
            if (scenario == 0) c.enabled = false;
            else if (scenario == 1) t.enabled = false;
            else if (scenario == 2) item.gameObject.SetActive(false);
            else if (scenario == 3) q.enabled = false;
            else if (scenario == 4) UnityEngine.Object.Destroy(c);
            else if (scenario == 5) UnityEngine.Object.Destroy(q);
            else throw new ArgumentException("Unknown handoff fault scenario.");
            Time.timeScale = 1; await Ticks(3);
            Require(sg.IsStopped && fg.IsStopped && JsonConvert.SerializeObject(t.Snapshot()) == records && t.ScaraProductId == item.ProductId &&
                t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-1", "WAIT-2" }), "Fault lost stop/record/slot/FIFO.");
            string reason = c != null ? c.FaultReason : null;
            if (c != null) Require(c.IsFaulted && c.HasControlledSequence, "Fault lost command ownership or did not latch.");
            if (scenario == 0) c.enabled = true;
            else if (scenario == 1) t.enabled = true;
            else if (scenario == 2) item.gameObject.SetActive(true);
            else if (scenario == 3) q.enabled = true;
            await Ticks(3);
            if (scenario != 2) Require(item.Owner == sg && sg.HeldItem == item, "Fault dropped confirmed holding.");
            else Require(item.Owner == null && sg.HeldItem == null && t.IsFaulted, "Lost holding was classified as normal residual.");
            if (q != null) Require(q.StepIndex == step && q.GripCount == 0 && q.ReleaseCount == 0, "Fault advanced/replayed the saved program.");
            Require(!l.Scara.MoveJoint(0, l.Scara.Axis1.CurrentPosition + 1) && !l.Fanuc.Joints[0].MoveTo(1) && !sg.TryRelease(out _), "Fault bypassed command gate.");
            if (c != null)
            {
                Require(!c.TryExecute(CellRunCommand.Start, out _) && !c.TryExecute(CellRunCommand.Resume, out _) &&
                    !c.TryReleaseControl(out _), "Fault permitted restart/release without acknowledgement.");
                Require(c.CanExecute(CellRunCommand.Reset, out _) == (scenario == 0 || scenario == 1 || scenario == 3), "Wrong consistency-checked explicit recovery eligibility.");
                Require(c.TryExecute(CellRunCommand.Stop, out _) && c.IsFaulted && c.FaultReason == reason, "Stop/re-enable cleared first fault.");
            }
            CheckSources(hashes);
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, savedStep = step, bothStopped = true,
                confirmedResidual = scenario == 2 ? 2 : 3, unconfirmedProducts = scenario == 2 ? 1 : 0,
                automaticallyRecovered = false, physicalOwnerPreserved = scenario != 2, retainedCommandOwnership = c != null,
                sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, hashes };
            Write("cell-run-handoff-fault-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.bothStopped, report.unconfirmedProducts, report.automaticallyRecovered });
        }
        finally { l.Scara.StopAll(); l.Fanuc.StopAll(); Time.timeScale = scale; }
    }

    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use a fresh unpaused LineWatch Play session.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity" && l.GetComponent<CellRunController>() != null, "Open LineWatch."); return l;
    }
    private static void AddWaiting(LineWatchCellLayout l, CellProductTracker t)
    {
        for (int i = 1; i <= 2; i++)
        {
            var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * (.4f * i);
            Require(t.TryRegister(item, "WAIT-" + i, ProductDestination.B, out var e), e);
        }
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
        var t = l.GetComponent<CellProductTracker>();
        return JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(),
            slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            owners = t.Snapshot().Select(r => new { r.Id, owner = t.TryGetItem(r.Id, out var item) && item.Owner != null ? item.Owner.GetInstanceID() : 0 }) });
    }
    private static string RobotSignature(LineWatchCellLayout l) => JsonConvert.SerializeObject(new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() }.Select(g =>
    {
        var q = g.GetComponent<RobotWaypointSequence>(); var item = g.GetComponent<RobotGripperLab>().TrialItem;
        return new { version = Version(g), q.State, q.StepIndex, q.GripCount, q.ReleaseCount,
            current = g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition } : g.Fanuc.Joints.Select(a => a.CurrentPosition).ToArray(),
            targets = g.Scara != null ? new[] { g.Scara.Axis1.TargetPosition, g.Scara.Axis2.TargetPosition, g.Scara.Axis3.TargetPosition, g.Scara.Axis4.TargetPosition } : g.Fanuc.Joints.Select(a => a.TargetPosition).ToArray(),
            itemPosition = item != null ? V(item.Body.position) : null, itemRotation = item != null ? Q(item.Body.rotation) : null, owner = item != null && item.Owner != null ? item.Owner.GetInstanceID() : 0 };
    }));
    private static string Signature(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.HasControlledSequence,
            controlled = c.ControlledSequence != null ? c.ControlledSequence.GetInstanceID() : 0, products = Products(l), robots = RobotSignature(l) });
    }
    private static void SetWaypoints(RobotWaypointSequence q, RobotWaypointSet set)
    { var so = new SerializedObject(q); so.FindProperty("waypoints").objectReferenceValue = set; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static void SetRetreat(RobotWaypointSet set, Vector3 position)
    { var so = new SerializedObject(set); so.FindProperty("retreat").FindPropertyRelative("positionMetres").vector3Value = position; so.ApplyModifiedPropertiesWithoutUndo(); }
    private static uint Version(RobotGripper g) => g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion;
    private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }
        .SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))).Replace("-", ""); }
    private static void CheckSources(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification changed source assets.");
    private static async Task Until(Func<bool> condition) { var watch = Stopwatch.StartNew(); while (!condition()) { Require(watch.Elapsed.TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static async Task Ticks(int count) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= count * Time.fixedDeltaTime); }
    private static void Write(string file, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + file, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
