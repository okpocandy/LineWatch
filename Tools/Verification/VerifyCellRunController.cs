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

public static class VerifyCellRunController
{
    public static async Task<object> Play()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var s = l.Scara; var f = l.Fanuc;
        var sg = s.GetComponent<RobotGripper>(); var fg = f.GetComponent<RobotGripper>();
        var sq = s.GetComponent<RobotWaypointSequence>(); var fq = f.GetComponent<RobotWaypointSequence>();
        GrippableItem registrationCandidate = null;
        var hashes = Sources(); var rows = new List<string>(); float scale = Time.timeScale;
        int transitions = 0, heldChecks = 0, completedSequences = 0;
        void Reject(Func<bool> request, string name)
        {
            string before = Signature(l);
            Require(!request() && Signature(l) == before, "Rejected request mutated state: " + name);
            rows.Add(name);
        }
        void RejectVoid(Action request, string name)
        {
            string before = Signature(l); request();
            Require(Signature(l) == before, "Blocked void command mutated state: " + name); rows.Add(name);
        }
        void Execute(CellRunCommand command, CellRunState expected)
        {
            string products = Products(l);
            Require(c.TryExecute(command, out var e), e);
            Require(c.State == expected && Products(l) == products && !t.IsFaulted, "Cell command changed product state."); transitions++;
        }
        void BlockLegacy(string phase, GrippableItem item)
        {
            string productError = null;
            Reject(() => t.TryRegister(registrationCandidate, "P-BLOCKED", ProductDestination.A, out productError), phase + "/manual register");
            Require(productError.Contains("수동 로봇 명령") && !registrationCandidate.IsTracked, "Registration bypassed admission.");
            Reject(() => t.TryFeedNext(out _, out productError), phase + "/manual feed");
            Require(productError.Contains("수동 로봇 명령"), "Feed bypassed admission.");
            Reject(() => t.TryRemove("P-000002", ProductLocation.Waiting, out productError), phase + "/manual remove");
            Require(productError.Contains("수동 로봇 명령"), "Removal bypassed admission.");
            Reject(() => s.MoveJoint(0, Next(s.Axis1)), phase + "/SCARA joint");
            Reject(() => f.MoveJoint(0, Next(f.Joints[0])), phase + "/FANUC joint");
            foreach (var a in new[] { s.Axis1, s.Axis2, s.Axis4 }.Concat(f.Joints))
                Reject(() => a.MoveTo(Next(a)), phase + "/raw rotary " + a.name);
            Reject(() => s.Axis3.MoveTo(.05f), phase + "/raw linear");
            RejectVoid(s.HomeAll, phase + "/SCARA home"); RejectVoid(f.HomeAll, phase + "/FANUC home");
            Reject(() => s.TryMoveTcp(s.ActualTcpPosition, s.ActualTcpYawDegrees, ScaraJointLab.IkSelection.Nearest, out _, out _), phase + "/SCARA TCP");
            Require(f.TryReadActualTcp(out var p, out var q, out var e), e);
            Reject(() => f.TryMoveTcp(p, q, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out _), phase + "/FANUC TCP");
            Reject(() => sq.TryStart(out _), phase + "/SCARA sequence start");
            Reject(() => fq.TryStart(out _), phase + "/FANUC sequence start");
            Reject(() => sq.TryResume(out _), phase + "/SCARA sequence resume");
            Reject(() => fq.TryResume(out _), phase + "/FANUC sequence resume");
            foreach (var g in new[] { sg, fg })
            {
                string error = null;
                Reject(() => g.TryGrip(item, out error), phase + "/grip " + g.name);
                Require(error.Contains("수동 로봇 명령"), "Grip failed for a reason other than cell admission.");
                Reject(() => g.TryRelease(out error), phase + "/release " + g.name);
                Require(error.Contains("수동 로봇 명령"), "Release bypassed cell admission.");
                var lab = g.GetComponent<RobotGripperLab>();
                Reject(() => lab.TryPrepare(out _), phase + "/trial prepare " + g.name);
                Reject(() => lab.TryMoveToWork(out _), phase + "/trial move " + g.name);
            }
        }
        try
        {
            Time.timeScale = 10;
            await Until(() => t.Count == 1 || t.IsFaulted);
            Require(c != null && c.State == CellRunState.Stopped && !t.IsFaulted && sg.IsStopped && fg.IsStopped, "Fresh stopped cell required.");
            Require(t.TryGetItem("P-000001", out var item), "Initial sample missing.");
            for (int i = 2; i <= 3; i++)
            {
                var waiting = UnityEngine.Object.Instantiate(l.ProductPrefab);
                waiting.Body.position = l.InputStop.position + Vector3.right * (.3f * i);
                Require(t.TryRegister(waiting, "P-00000" + i, i == 2 ? ProductDestination.B : ProductDestination.A, out var e), e);
            }
            var fifo = t.WaitingSnapshot().ToArray(); string initialProducts = Products(l);
            registrationCandidate = UnityEngine.Object.Instantiate(l.ProductPrefab);
            registrationCandidate.Body.position = l.InputStop.position + Vector3.right * 1.2f;
            Execute(CellRunCommand.Start, CellRunState.Running);
            Require(sg.IsStopped && fg.IsStopped, "Common Start must not dispatch a picking program in step 2-a.");
            Reject(() => c.TryExecute(CellRunCommand.Start, out _), "duplicate common start");
            BlockLegacy("running", item);
            Execute(CellRunCommand.Pause, CellRunState.Paused);
            Reject(() => c.TryExecute(CellRunCommand.Start, out _), "start while paused");
            BlockLegacy("paused", item);
            Execute(CellRunCommand.Resume, CellRunState.Running);
            Execute(CellRunCommand.Stop, CellRunState.Stopped);
            Execute(CellRunCommand.Stop, CellRunState.Stopped);
            Reject(() => c.TryExecute(CellRunCommand.Reset, out _), "acknowledgement outside fault");
            Require(Products(l) == initialProducts, "State transitions altered inventory.");

            Time.timeScale = .1f;
            Require(s.MoveJoint(0, 15f) && f.MoveJoint(0, 15f), "Stopped manual commands denied.");
            await Ticks(1);
            Require(!sg.IsStopped && !fg.IsStopped, "Both manual motions must still be active before stopping.");
            Reject(() => c.TryExecute(CellRunCommand.Start, out _), "start during manual movement");
            Execute(CellRunCommand.Stop, CellRunState.Stopped);
            var stoppedPose = Positions(l); await Ticks(3);
            Require(stoppedPose.SequenceEqual(Positions(l)) && sg.IsStopped && fg.IsStopped, "Common Stop did not freeze both robots.");
            Time.timeScale = 10;
            Require(s.MoveJoint(0, .015f), "Small-yaw fixture rejected."); await Until(() => sg.IsStopped);
            Require(s.TryReadFkDiagnostics(out var smallYaw, out var smallError) && smallYaw.YawErrorDegrees < .001f &&
                Mathf.Abs(s.ActualTcpYawDegrees - .015f) < .001f, smallError ?? "Small nonzero yaw collapsed to zero.");

            foreach (var sequence in new[] { sq, fq })
            {
                Require(sequence.TryStart(out var e), e);
                await Until(() => sequence.StepIndex >= 1 || sequence.State == RobotWaypointSequence.RunState.Faulted);
                Require(sequence.IsBusy && !c.IsFaulted, "Sequence failed before interruption check.");
                Reject(() => s.MoveJoint(0, Next(s.Axis1)), "SCARA manual during " + sequence.name);
                Reject(() => f.MoveJoint(0, Next(f.Joints[0])), "FANUC manual during " + sequence.name);
                Reject(() => s.Axis1.MoveTo(Next(s.Axis1)), "raw SCARA during " + sequence.name);
                Reject(() => f.Joints[0].MoveTo(Next(f.Joints[0])), "raw FANUC during " + sequence.name);
                RejectVoid(s.HomeAll, "SCARA home during " + sequence.name);
                RejectVoid(f.HomeAll, "FANUC home during " + sequence.name);
                var other = sequence == sq ? fq : sq;
                Reject(() => other.TryStart(out _), "other sequence during " + sequence.name);
                Reject(() => c.TryExecute(CellRunCommand.Start, out _), "cell start during " + sequence.name);
                int step = sequence.StepIndex;
                Execute(CellRunCommand.Stop, CellRunState.Stopped);
                Require(sequence.State == RobotWaypointSequence.RunState.Paused && sequence.StepIndex == step, "Stop discarded individual progress.");
                Reject(() => c.TryExecute(CellRunCommand.Start, out _), "cell start with saved individual path");
                Require(sequence.TryResume(out e), e);
                await Until(() => !sequence.IsBusy);
                Require(sequence.State == RobotWaypointSequence.RunState.Completed && !c.IsFaulted, sequence.LastError ?? c.FaultReason);
                completedSequences++;
            }

            await MoveToProduct(sg, item, l.InputStop);
            Require(sg.TryGrip(item, out var gripError), gripError);
            string held = Products(l); var bodyPosition = item.Body.position;
            foreach (var cmd in new[] { CellRunCommand.Start, CellRunCommand.Pause, CellRunCommand.Resume, CellRunCommand.Stop, CellRunCommand.Stop })
            {
                Require(c.TryExecute(cmd, out var e), e); await Ticks(2);
                Require(Products(l) == held && item.Owner == sg && sg.HeldItem == item && t.ScaraProductId == item.ProductId &&
                    Vector3.Distance(item.Body.position, bodyPosition) < .00001f && t.WaitingSnapshot().SequenceEqual(fifo), "Held state/FIFO changed on " + cmd);
                heldChecks++;
            }
            Require(!c.IsFaulted && !t.IsFaulted && t.Count == 3 && t.InputProductId == null && t.HandoffProductId == null && t.FanucProductId == null, "Normal accounting mismatch.");
            CheckSources(hashes);
            var report = new { passed = true, verifiedUtc = DateTime.UtcNow, transitions, rejectedRequestsPreserved = rows.Count,
                completedSequences, heldChecks, registeredProducts = t.Count, physicallyConfirmedResidual = 3, unconfirmedProducts = 0,
                waitingOrder = fifo, commonStartRobotMoves = 0, commonResumeRobotMoves = 0, sourcesPreserved = hashes.Count,
                directButtonClicks = 0, callbacksInjected = 0, hashes, rows,
                scope = "LW-11 step 2-a: actual legacy command gates/common stop and state permission. No automatic cell product dispatch or common path resume." };
            Write("cell-run-controller-play.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.transitions, report.rejectedRequestsPreserved, report.completedSequences,
                report.heldChecks, report.registeredProducts, report.physicallyConfirmedResidual, report.unconfirmedProducts, report.sourcesPreserved });
        }
        finally { c.TryExecute(CellRunCommand.Stop, out _); Time.timeScale = scale; }
    }

    public static async Task<object> Lifecycle(int scenario)
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var sg = l.Scara.GetComponent<RobotGripper>(); var fg = l.Fanuc.GetComponent<RobotGripper>();
        float scale = Time.timeScale; var hashes = Sources();
        try
        {
            Time.timeScale = 10; await Until(() => t.Count == 1 || t.IsFaulted);
            Require(t.TryGetItem("P-000001", out var item) && !t.IsFaulted, "Fresh sample required.");
            await MoveToProduct(sg, item, l.InputStop); Require(sg.TryGrip(item, out var e), e);
            for (int i = 2; i <= 3; i++)
            {
                var waiting = UnityEngine.Object.Instantiate(l.ProductPrefab);
                waiting.Body.position = l.InputStop.position + Vector3.right * (.3f * i);
                Require(t.TryRegister(waiting, "P-00000" + i, ProductDestination.B, out e), e);
            }
            var fifo = t.WaitingSnapshot().ToArray();
            var lastRecords = t.Snapshot().ToArray();
            string records = JsonConvert.SerializeObject(lastRecords);
            Time.timeScale = .1f;
            float scaraTarget = l.Scara.Axis1.CurrentPosition < l.Scara.Axis1.MaximumDegrees - 15 ? l.Scara.Axis1.CurrentPosition + 12 : l.Scara.Axis1.CurrentPosition - 12;
            Require(l.Scara.MoveJoint(0, scaraTarget) && l.Fanuc.MoveJoint(0, 15f), "Moving fault fixture failed.");
            await Ticks(1);
            Require(!sg.IsStopped && !fg.IsStopped, "Both motions must still be active before the fault.");
            if (scenario == 0) t.enabled = false;
            else if (scenario == 1) c.enabled = false;
            else if (scenario == 2) UnityEngine.Object.Destroy(c);
            else if (scenario == 3) item.gameObject.SetActive(false);
            else if (scenario == 4) UnityEngine.Object.Destroy(t);
            else throw new ArgumentException("Unknown lifecycle scenario.");
            await Ticks(3);
            Require(sg.IsStopped && fg.IsStopped, "Fault did not stop both robots.");
            var stopped = Positions(l);
            string reason = c != null ? c.FaultReason : null;
            if (c != null) Require(c.IsFaulted && !string.IsNullOrWhiteSpace(reason), "Cell fault not latched.");
            if (scenario == 0) t.enabled = true;
            if (scenario == 1) c.enabled = true;
            if (scenario == 3) item.gameObject.SetActive(true);
            await Ticks(3);
            Require(stopped.SequenceEqual(Positions(l)), "Reactivation resumed motion.");
            if (t != null) Require(JsonConvert.SerializeObject(t.Snapshot()) == records && t.ScaraProductId == item.ProductId &&
                t.WaitingSnapshot().SequenceEqual(fifo), "Fault discarded last records/slot/FIFO.");
            if (scenario != 3) Require(item.Owner == sg && sg.HeldItem == item, "Manager lifecycle dropped physical product.");
            else Require(item.Owner == null && sg.HeldItem == null && t.IsFaulted, "Lost holding was mistaken for confirmed holding.");
            string before = Signature(l);
            Require(!l.Scara.MoveJoint(0, Next(l.Scara.Axis1)) && !l.Fanuc.MoveJoint(0, Next(l.Fanuc.Joints[0])) &&
                !l.Scara.Axis1.MoveTo(Next(l.Scara.Axis1)) && !l.Fanuc.Joints[0].MoveTo(Next(l.Fanuc.Joints[0])) &&
                !sg.TryRelease(out _) && !sg.GetComponent<RobotGripperLab>().TryPrepare(out _), "Lifecycle bypassed command gate.");
            Require(Signature(l) == before, "Rejected lifecycle commands changed state.");
            if (c != null)
            {
                Require(!c.TryExecute(CellRunCommand.Start, out _) && !c.TryExecute(CellRunCommand.Resume, out _), "Faulted restart accepted without acknowledgement.");
                Require(c.CanExecute(CellRunCommand.Reset, out _) == (scenario == 0 || scenario == 1), "Wrong consistency-checked explicit recovery eligibility.");
                Require(c.TryExecute(CellRunCommand.Stop, out _) && c.IsFaulted && c.FaultReason == reason, "Stop/re-enable cleared fault reason.");
            }
            CheckSources(hashes);
            var unconfirmed = lastRecords.Where(r => t == null || !t.TryGetItem(r.Id, out var actual) || !actual.IsReady || actual.ProductTracker != t ||
                actual.Owner != (r.Location == ProductLocation.Scara ? sg : null) ||
                r.Location == ProductLocation.Scara && sg.HeldItem != actual).Select(r => r.Id).ToArray();
            Require(unconfirmed.Length == (scenario == 4 ? 3 : scenario == 3 ? 1 : 0), "Unexpected physical accounting after lifecycle fault.");
            var report = new { passed = true, scenario, verifiedUtc = DateTime.UtcNow, bothRobotsStopped = true, lastRecordsRetained = t != null,
                physicalOwnerPreserved = scenario != 3, physicallyConfirmedResidual = lastRecords.Length - unconfirmed.Length,
                unconfirmedProducts = unconfirmed.Length, unconfirmedIds = unconfirmed,
                waitingOrder = fifo, automaticRecovery = false, managerRemovalBypass = false, sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, hashes };
            Write("cell-run-controller-lifecycle-" + scenario + ".json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.bothRobotsStopped, report.physicalOwnerPreserved, report.unconfirmedProducts, report.automaticRecovery });
        }
        finally { l.Scara.StopAll(); l.Fanuc.StopAll(); Time.timeScale = scale; }
    }

    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh unpaused LineWatch Play Mode.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch."); return l;
    }
    private static string Products(LineWatchCellLayout l)
    {
        var t = l.GetComponent<CellProductTracker>(); var sg = l.Scara.GetComponent<RobotGripper>(); var fg = l.Fanuc.GetComponent<RobotGripper>();
        return JsonConvert.SerializeObject(new { records = t != null ? t.Snapshot() : null, waiting = t != null ? t.WaitingSnapshot() : null,
            slots = t != null ? new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId } : null,
            held = new[] { sg.HeldItem != null ? sg.HeldItem.ProductId : null, fg.HeldItem != null ? fg.HeldItem.ProductId : null },
            owners = t != null ? t.Snapshot().Select(r => new { r.Id, owner = t.TryGetItem(r.Id, out var item) && item.Owner != null ? item.Owner.GetInstanceID() : 0 }).ToArray() : null });
    }
    private static string Signature(LineWatchCellLayout l) => JsonConvert.SerializeObject(new { products = Products(l), positions = Positions(l),
        versions = new[] { l.Scara.CommandVersion, l.Fanuc.CommandVersion },
        targets = new[] { l.Scara.Axis1.TargetPosition, l.Scara.Axis2.TargetPosition, l.Scara.Axis3.TargetPosition, l.Scara.Axis4.TargetPosition }.Concat(l.Fanuc.Joints.Select(a => a.TargetPosition)),
        sequences = new[] { l.Scara.GetComponent<RobotWaypointSequence>(), l.Fanuc.GetComponent<RobotWaypointSequence>() }.Select(q => new { q.State, q.StepIndex, q.GripCount, q.ReleaseCount }),
        state = l.GetComponent<CellRunController>()?.State, fault = l.GetComponent<CellRunController>()?.FaultReason });
    private static float[] Positions(LineWatchCellLayout l) => new[] { l.Scara.Axis1.CurrentPosition, l.Scara.Axis2.CurrentPosition, l.Scara.Axis3.CurrentPosition, l.Scara.Axis4.CurrentPosition }
        .Concat(l.Fanuc.Joints.Select(a => a.CurrentPosition)).ToArray();
    private static float Next(RotaryAxis a) => a.CurrentPosition < a.MaximumDegrees - 3 ? a.CurrentPosition + 2 : a.CurrentPosition - 2;
    private static async Task MoveToProduct(RobotGripper g, GrippableItem item, Transform station)
    {
        var point = item.GetGripPoint(RobotWaypointSet.RobotType.Scara);
        var rotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = station.TransformPoint(point.localPosition) - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        Require(g.Scara.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position),
            Vector3.SignedAngle(Vector3.forward, (Quaternion.Inverse(g.RobotFrame.rotation) * rotation) * Vector3.forward, Vector3.up),
            ScaraJointLab.IkSelection.PositiveElbow, out _, out var e), e);
        await Until(() => g.IsStopped); await Ticks(2);
    }
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }
        .SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static void CheckSources(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification changed source assets.");
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))).Replace("-", ""); }
    private static async Task Ticks(int count) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= count * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var watch = Stopwatch.StartNew(); while (!condition()) { Require(watch.Elapsed.TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool value, string error) { if (!value) throw new InvalidOperationException(error); }
}
