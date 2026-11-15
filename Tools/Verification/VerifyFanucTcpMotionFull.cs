using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using Policy = LineWatch.FanucJointLab.IkSeedPolicy;

public static class VerifyFanucTcpMotionFull
{
    private const int RandomSeed = 20261115;
    private const Policy SeedPolicy = Policy.CurrentThenFixed;
    private const string DirectoryPath = "Logs/LW07D";

    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused && Time.timeScale > 0f, "Use unpaused RobotLab Play Mode.");
        var lab = UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        Require(lab != null && lab.IsReady, "FANUC references required.");
        var axes = lab.Joints.ToArray();
        var lower = axes.Select(j => j.MinimumDegrees).ToArray();
        var upper = axes.Select(j => j.MaximumDegrees).ToArray();
        var speeds = axes.Select(j => j.SpeedDegreesPerSecond).ToArray();
        var poses = Poses(lower, upper);
        var goals = poses.Select(q => FanucForwardKinematics.Calculate(q, lab.Geometry)).ToArray();
        var token = lab.destroyCancellationToken;
        float originalScale = Time.timeScale, originalStep = Time.fixedDeltaTime;
        bool originalAxisEnabled = axes[5].enabled;
        var paths = new[] { "Assets/01.Scenes/RobotLab.unity", "Assets/03.Prefabs/FanucRobot_Work.prefab",
            "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset",
            "Assets/02.Scripts/LineWatch/FanucJointLab.cs", "Assets/02.Scripts/LineWatch/RotaryAxis.cs",
            "Assets/02.Scripts/LineWatch/FanucForwardKinematics.cs", "Assets/02.Scripts/LineWatch/FanucInverseKinematics.cs",
            "Tools/Verification/VerifyFanucTcpMotionFull.cs" };
        var hashes = paths.ToDictionary(p => p, Hash);
        string poseJson = JsonConvert.SerializeObject(poses);
        bool? priorPosesMatch = null;
        const string priorReport = "Logs/LW07C/full-convergence-final.json";
        if (File.Exists(priorReport))
        {
            var prior = JsonConvert.DeserializeObject<Newtonsoft.Json.Linq.JObject>(File.ReadAllText(priorReport));
            priorPosesMatch = poseJson == JsonConvert.SerializeObject(prior["jointPoses"].ToObject<float[][]>());
            Require(priorPosesMatch.Value, "Target poses differ from the prior pure IK campaign.");
        }
        var plan = new { version = 1, randomSeed = RandomSeed, policy = SeedPolicy.ToString(), lower, upper, speeds,
            initialJointsDegrees = new[] { 5f, -8f, 12f, 6f, 9f, -4f }, jointPoses = poses,
            targets = goals.Select((g, i) => new { index = i, metres = V(g.PositionMetres), quaternion = Q(g.Rotation) }).ToArray(),
            positionToleranceMetres = .002f, rotationToleranceDegrees = .5f,
            solverMaximumIterations = 240, solverMaximumMilliseconds = 50,
            simulationTimeScale = 10f, fixedDeltaTime = originalStep, sampleClock = "Time.fixedTimeAsDouble", sourceHashes = hashes };
        string planJson = JsonConvert.SerializeObject(plan, Formatting.Indented);
        Directory.CreateDirectory(DirectoryPath);
        File.WriteAllText(DirectoryPath + "/full-motion-plan.json", planJson);
        var rows = new List<Row>();
        int samples = 0, rejections = 0, arrivals = 0, movingGoals = 0;
        float maxFkPosition = 0f, maxFkRotation = 0f, maxSpeed = 0f, maxSpread = 0f;
        bool stopHeld = false, disabledAxisStopped = false, resumesFromActual = true;
        string fatal = null;
        var wall = Stopwatch.StartNew();
        double simulationStart = Time.fixedTimeAsDouble;
        float[] Current() => axes.Select(j => j.CurrentPosition).ToArray();

        void Reject(Func<bool> action)
        {
            string before = State(lab, axes);
            Require(lab.IsTcpMoving && !action() && State(lab, axes) == before, "Invalid replacement changed the active plan.");
            rejections++;
        }
        bool Accept(int index, out FanucJointLab.IkPreview preview, out string error)
        {
            var before = Current(); var goal = goals[index];
            bool accepted = lab.TryMoveTcp(goal.PositionMetres, goal.Rotation, SeedPolicy, out preview, out error);
            Require(before.SequenceEqual(Current()), "A request teleported an axis.");
            if (accepted) resumesFromActual &= preview.CurrentJointDegrees.SequenceEqual(before);
            return accepted;
        }
        async Task Hold()
        {
            var q = Current(); double before = Time.fixedTimeAsDouble;
            await Until(() => Time.fixedTimeAsDouble - before >= 4 * Time.fixedDeltaTime, token);
            Require(q.SequenceEqual(Current()) && axes.All(j => !j.IsMoving), "Stopped axes drifted.");
        }
        async Task Advance()
        {
            var q = Current();
            await Until(() => !q.SequenceEqual(Current()) || !lab.IsTcpMoving, token);
            Require(lab.IsTcpMoving, "Interruption fixture finished before the interruption.");
        }
        async Task Trace(float[] start, FanucJointLab.IkPreview accepted)
        {
            var end = accepted.Result.JointDegrees.ToArray();
            var previous = Current(); double previousTime = Time.fixedTimeAsDouble;
            var timeout = Stopwatch.StartNew();
            do
            {
                await Task.Delay(1, token);
                Require(timeout.Elapsed.TotalSeconds < 15 && string.IsNullOrEmpty(lab.LastMotionError), "Motion timed out or aborted: " + lab.LastMotionError);
                var q = Current(); double now = Time.fixedTimeAsDouble; float min = 1, max = 0; int changed = 0;
                for (int i = 0; i < 6; i++)
                {
                    Require(q[i] >= lower[i] && q[i] <= upper[i], "Intermediate angle outside saved limits.");
                    if (now > previousTime)
                    {
                        float speed = (float)(Mathf.Abs(q[i] - previous[i]) / (now - previousTime));
                        maxSpeed = Mathf.Max(maxSpeed, speed);
                        Require(speed <= speeds[i] + .02f, "Joint speed exceeded its configured bound.");
                    }
                    if (Mathf.Abs(end[i] - start[i]) > .01f)
                    {
                        float t = (q[i] - start[i]) / (end[i] - start[i]);
                        min = Mathf.Min(min, t); max = Mathf.Max(max, t); changed++;
                    }
                }
                if (changed > 1)
                {
                    maxSpread = Mathf.Max(maxSpread, max - min);
                    Require(max - min < .0001f, "Joint progress lost synchronization.");
                }
                Require(lab.TryReadFkDiagnostics(out var d, out var e), "In-flight FK unavailable: " + e);
                maxFkPosition = Mathf.Max(maxFkPosition, d.PositionErrorMetres);
                maxFkRotation = Mathf.Max(maxFkRotation, d.RotationErrorDegrees);
                Require(d.PositionErrorMetres < .00001f && d.RotationErrorDegrees < .005f, "In-flight FK/model mismatch.");
                samples++; previous = q; previousTime = now;
            } while (lab.IsTcpMoving);
            Require(axes.All(j => !j.IsMoving && j.IsAtTarget), "Not all six axes completed.");
        }
        try
        {
            Time.timeScale = 10f;
            lab.StopAll();
            for (int i = 0; i < 6; i++) Require(lab.MoveJoint(i, plan.initialJointsDegrees[i]), "Initial pose request rejected.");
            await Until(() => axes.All(j => !j.IsMoving && j.IsAtTarget), token);
            simulationStart = Time.fixedTimeAsDouble;
            for (int index = 0; index < goals.Length; index++)
            {
                var row = new Row { index = index, sourceJoints = poses[index], startJoints = Current(),
                    targetMetres = V(goals[index].PositionMetres), targetQuaternion = Q(goals[index].Rotation) };
                rows.Add(row);
                string before = State(lab, axes);
                row.accepted = Accept(index, out var accepted, out var error);
                row.error = error;
                if (accepted.Result != null)
                {
                    row.reason = accepted.Result.Reason.ToString(); row.iterations = accepted.Result.Iterations;
                    row.seedsTried = accepted.Result.SeedsTried; row.seedName = accepted.SeedName;
                    row.solveMilliseconds = accepted.Result.ElapsedMilliseconds;
                    row.positionResidualMetres = accepted.Result.PositionErrorMetres;
                    row.rotationResidualDegrees = accepted.Result.RotationErrorDegrees;
                }
                if (!row.accepted)
                {
                    Require(before == State(lab, axes), "A rejected feasible goal changed commands.");
                    continue; // Preserve failures in the fixed campaign; do not filter or retry them.
                }
                row.startedMoving = lab.IsTcpMoving;
                if (row.startedMoving) movingGoals++;
                if (index == 1) Reject(() => lab.TryMoveTcp(new Vector3(3, 0, 0), goals[index].Rotation, SeedPolicy, out _, out _));
                if (index == 33) Reject(() => lab.TryMoveTcp(goals[index].PositionMetres, default, SeedPolicy, out _, out _));
                if (index == 66) Reject(() => lab.TryMoveTcp(new Vector3(float.NaN, 0, 0), goals[index].Rotation, SeedPolicy, out _, out _));
                if (index == 99) Reject(() => lab.TryMoveTcp(goals[index].PositionMetres, goals[index].Rotation, (Policy)999, out _, out _));
                if (index == 35)
                {
                    await Advance(); lab.StopAll(); Require(!lab.IsTcpMoving, "Stop did not cancel the plan.");
                    await Hold(); stopHeld = true;
                    Require(Accept(index, out accepted, out error), "Stopped goal could not resume: " + error);
                    row.interruption = "stop/resume";
                }
                if (index == 75)
                {
                    await Advance(); axes[5].enabled = false;
                    await Until(() => !lab.IsTcpMoving, token);
                    Require(!string.IsNullOrEmpty(lab.LastMotionError) && axes.All(j => !j.IsMoving), "Disabled axis did not stop the owned group.");
                    await Hold(); disabledAxisStopped = true; axes[5].enabled = originalAxisEnabled;
                    Require(Accept(index, out accepted, out error), "Fault recovery could not resume: " + error);
                    row.interruption = "disabled axis/recover/resume";
                }
                row.commandedJoints = accepted.Result.JointDegrees.ToArray();
                await Trace(Current(), accepted);
                Require(lab.TryReadActualTcp(out var position, out var rotation, out error), error);
                row.actualMetres = V(position); row.actualQuaternion = Q(rotation); row.arrivedJoints = Current();
                row.actualPositionErrorMetres = Vector3.Distance(position, goals[index].PositionMetres);
                row.actualRotationErrorDegrees = Angle(rotation, goals[index].Rotation);
                row.arrived = row.actualPositionErrorMetres <= .002f && row.actualRotationErrorDegrees <= .5f;
                if (row.arrived) arrivals++;
            }
        }
        catch (Exception e) { fatal = e.ToString(); }
        finally
        {
            Time.timeScale = originalScale; Time.fixedDeltaTime = originalStep;
            if (lab != null && Application.isPlaying) { axes[5].enabled = originalAxisEnabled; lab.StopAll(); }
        }
        bool unchanged = hashes.All(p => Hash(p.Key) == p.Value);
        bool planUnchanged = File.ReadAllText(DirectoryPath + "/full-motion-plan.json") == planJson;
        bool passed = fatal == null && arrivals == 100 && rejections == 4 && stopHeld && disabledAxisStopped && resumesFromActual && unchanged && planUnchanged;
        var report = new { passed, utc = DateTime.UtcNow.ToString("o"), fatal, actualArrivals = arrivals,
            requestedTargets = rows.Count, movingGoals, noMotionGoals = rows.Count(r => r.accepted && !r.startedMoving),
            rejectedFeasibleGoals = rows.Count(r => !r.accepted), rejectedRequestsPreserved = rejections,
            movingSamples = samples, maxActualPositionErrorMetres = rows.Where(r => r.arrived).Select(r => r.actualPositionErrorMetres).DefaultIfEmpty().Max(),
            maxActualRotationErrorDegrees = rows.Where(r => r.arrived).Select(r => r.actualRotationErrorDegrees).DefaultIfEmpty().Max(),
            maxMovingFkErrorMetres = maxFkPosition, maxMovingFkRotationErrorDegrees = maxFkRotation,
            maxObservedJointSpeedDegreesPerSecond = maxSpeed, maxJointProgressSpread = maxSpread,
            stopHeld, disabledAxisStopped, resumesFromActual, sourceAssetsAndCodeUnchanged = unchanged,
            planUnchanged, planSha256 = HashText(planJson), posesSha256 = HashText(poseJson), priorPureIkPosesMatch = priorPosesMatch,
            elapsedWallSeconds = wall.Elapsed.TotalSeconds, simulatedSeconds = Time.fixedTimeAsDouble - simulationStart,
            selectedSeeds = rows.Where(r => r.accepted).GroupBy(r => r.seedName).Select(g => new { name = g.Key, count = g.Count() }).ToArray(),
            fixedGoalFallbacks = rows.Count(r => r.accepted && r.seedName != "현재 자세"), directButtonClicks = 0,
            scope = "100 fixed known-feasible target requests through real FixedUpdate axes and measured TCP; 10x simulation, no oracle IK seeds, no collision/acceleration/industrial safety or global convergence claim", rows };
        File.WriteAllText(DirectoryPath + "/full-motion-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return new { report.passed, report.actualArrivals, report.requestedTargets, report.movingGoals, report.noMotionGoals,
            report.rejectedFeasibleGoals, report.rejectedRequestsPreserved, report.movingSamples, report.maxActualPositionErrorMetres,
            report.maxActualRotationErrorDegrees, report.maxObservedJointSpeedDegreesPerSecond, report.maxJointProgressSpread,
            report.stopHeld, report.disabledAxisStopped, report.resumesFromActual, report.sourceAssetsAndCodeUnchanged, report.planUnchanged,
            report.priorPureIkPosesMatch, report.selectedSeeds, report.elapsedWallSeconds, report.simulatedSeconds,
            failedGoals = rows.Where(r => !r.arrived).Select(r => new { r.index, r.reason, r.error, r.iterations }), report.fatal };
    }

    private sealed class Row
    {
        public int index, iterations, seedsTried;
        public bool accepted, startedMoving, arrived;
        public string error, reason, seedName, interruption;
        public double solveMilliseconds;
        public float? positionResidualMetres, rotationResidualDegrees;
        public float actualPositionErrorMetres, actualRotationErrorDegrees;
        public float[] sourceJoints, startJoints, commandedJoints, arrivedJoints, targetMetres, targetQuaternion, actualMetres, actualQuaternion;
    }
    private static List<float[]> Poses(float[] lower, float[] upper)
    {
        var result = new List<float[]> { new float[6] };
        for (int i = 0; i < 6; i++) foreach (float value in new[] { lower[i], upper[i] })
        { var q = new float[6]; q[i] = value; result.Add(q); }
        result.Add((float[])lower.Clone()); result.Add((float[])upper.Clone());
        result.Add(Enumerable.Range(0, 6).Select(i => i % 2 == 0 ? lower[i] : upper[i]).ToArray());
        result.Add(Enumerable.Range(0, 6).Select(i => i % 2 == 0 ? upper[i] : lower[i]).ToArray());
        result.Add(Enumerable.Repeat(.0001f, 6).ToArray()); result.Add(Enumerable.Repeat(-.0001f, 6).ToArray());
        result.Add(new[] { 25f, -30f, 20f, -35f, 15f, 40f }); result.Add(new[] { -40f, 15f, -25f, 30f, -35f, -20f });
        var random = new System.Random(RandomSeed);
        while (result.Count < 100) result.Add(Enumerable.Range(0, 6).Select(i => lower[i] + (upper[i] - lower[i]) * (float)random.NextDouble()).ToArray());
        Require(result.Select(q => JsonConvert.SerializeObject(q)).Distinct().Count() == 100, "Duplicate source poses.");
        Require(result.All(q => q.Select((v, i) => v >= lower[i] && v <= upper[i]).All(v => v)), "Source pose outside limits.");
        return result;
    }
    private static string State(FanucJointLab lab, RotaryAxis[] axes) => JsonConvert.SerializeObject(new { lab.IsTcpMoving,
        lab.LastMotionError, axes = axes.Select(j => new { j.CurrentPosition, j.TargetPosition, j.IsMoving, rotation = Q(j.transform.localRotation) }) });
    private static float[] V(Vector3 v) => new[] { v.x, v.y, v.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static async Task Until(Func<bool> condition, CancellationToken token)
    {
        var watch = Stopwatch.StartNew();
        while (!condition()) { token.ThrowIfCancellationRequested(); Require(watch.Elapsed.TotalSeconds < 15, "Verification wait timed out."); await Task.Delay(1, token); }
    }
    private static float Angle(Quaternion a, Quaternion b)
    { var d = Quaternion.Inverse(a) * b; return (float)(2 * Math.Atan2(Math.Sqrt((double)d.x * d.x + (double)d.y * d.y + (double)d.z * d.z), Math.Abs(d.w)) * 180 / Math.PI); }
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static string HashText(string s) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(System.Text.Encoding.UTF8.GetBytes(s))); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
