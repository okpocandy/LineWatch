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
using IK = LineWatch.ScaraInverseKinematics;

public static class VerifyScaraInverseKinematicsFull
{
    private const int Seed = 20261114;
    private static readonly string[] SourcePaths = { "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/01.Scenes/RobotLab.unity", "ProjectSettings/ProjectSettings.asset" };
    private static IK.JointLimits Limits => new IK.JointLimits(new IK.JointValues(-170f,-150f,0f,-180f), new IK.JointValues(170f,150f,.15f,180f));

    public static object Static()
    {
        Require(!EditorApplication.isPlayingOrWillChangePlaymode, "Edit Mode required");
        var hashes = SourcePaths.ToDictionary(path => path, Hash);
        var root = PrefabUtility.LoadPrefabContents(SourcePaths[0]);
        try
        {
            var frame = root.transform.Find("RobotFrame");
            var a1 = root.transform.Find("Base/Axis1");
            var a2 = a1.Find("Axis2");
            var a3 = a2.Find("Axis3");
            var a4 = a3.Find("Axis4");
            var gripper = a4.Find("Gripper");
            var tcp = gripper.Find("TCP");
            var h1 = a1.localRotation; var h2 = a2.localRotation; var h3 = a3.localPosition; var h4 = a4.localRotation;
            var hg = gripper.localPosition; var hgr = gripper.localRotation;
            var rootPosition = root.transform.position; var rootRotation = root.transform.rotation;
            var placements = new[] {
                new { name = "Authored", position = rootPosition, rotation = rootRotation },
                new { name = "Translated and yawed", position = new Vector3(2.1f,1.2f,-3f), rotation = Quaternion.Euler(0f,37f,0f) },
                new { name = "Translated and tilted", position = new Vector3(-4.2f,.8f,2.3f), rotation = Quaternion.Euler(20f,47f,-15f) }
            };
            var tools = new[] {
                new { name = "Original", delta = Vector3.zero, offset = new Vector3(0f,-.034177f,0f), yaw = 0f },
                new { name = "Mount 20mm lower", delta = new Vector3(0f,-.02f,0f), offset = new Vector3(0f,-.054177f,0f), yaw = 0f },
                new { name = "Horizontal offset", delta = new Vector3(-.01f,0f,.02f), offset = new Vector3(.02f,-.034177f,.01f), yaw = 0f },
                new { name = "Combined mount and yaw", delta = new Vector3(-.01f,-.02f,.02f), offset = new Vector3(.02f,-.054177f,.01f), yaw = 35f }
            };
            var poses = Poses();
            var groups = new List<object>();
            var failures = new List<object>();
            int targets = 0, candidates = 0;
            float maxFk = 0f, maxLocal = 0f, maxWorld = 0f, maxYaw = 0f, maxRotation = 0f;
            foreach (var tool in tools)
            {
                gripper.localPosition = hg + tool.delta;
                gripper.localRotation = hgr * Quaternion.AngleAxis(tool.yaw,Vector3.up);
                var geometry = Geometry(tool.offset,tool.yaw);
                foreach (var placement in placements)
                {
                    root.transform.SetPositionAndRotation(placement.position,placement.rotation);
                    int groupCandidates = 0, groupFailed = 0;
                    for (int i = 0; i < poses.Count; i++)
                    {
                        var p = poses[i];
                        var goal = ScaraForwardKinematics.Calculate(p.x,p.y,p.z,p.w,geometry);
                        // Include equivalent yaw representations without changing the goal direction.
                        float yaw = goal.YawDegrees + (i % 3 - 1) * 360f;
                        var result = IK.Solve(goal.PositionMetres,yaw,geometry,Limits);
                        targets++;
                        if (!result.Success)
                        {
                            failures.Add(new { tool = tool.name, placement = placement.name, index = i, source = Values(p), reason = result.Failure.ToString() });
                            groupFailed++;
                            continue;
                        }
                        foreach (var solution in result.Solutions)
                        {
                            var j = solution.Joints;
                            Require(InRange(j), "Returned joint out of bounds");
                            a1.localRotation = h1 * Quaternion.AngleAxis(j.Q1Degrees,Vector3.up);
                            a2.localRotation = h2 * Quaternion.AngleAxis(j.Q2Degrees,Vector3.up);
                            a3.localPosition = h3 + Vector3.down * j.D3Metres;
                            a4.localRotation = h4 * Quaternion.AngleAxis(j.Q4Degrees,Vector3.up);
                            var fk = ScaraForwardKinematics.Calculate(j.Q1Degrees,j.Q2Degrees,j.D3Metres,j.Q4Degrees,geometry);
                            float ef = Vector3.Distance(fk.PositionMetres,goal.PositionMetres);
                            float el = Vector3.Distance(frame.InverseTransformPoint(tcp.position),goal.PositionMetres);
                            float ew = Vector3.Distance(tcp.position,frame.TransformPoint(goal.PositionMetres));
                            float ey = Mathf.Abs(Mathf.DeltaAngle(Vector3.SignedAngle(frame.forward,tcp.forward,frame.up),goal.YawDegrees));
                            float er = RotationError(frame.rotation * Quaternion.AngleAxis(goal.YawDegrees,Vector3.up),tcp.rotation);
                            maxFk = Mathf.Max(maxFk,ef); maxLocal = Mathf.Max(maxLocal,el); maxWorld = Mathf.Max(maxWorld,ew);
                            maxYaw = Mathf.Max(maxYaw,ey); maxRotation = Mathf.Max(maxRotation,er);
                            candidates++; groupCandidates++;
                            if (ef > .00001f || el > .00001f || ew > .00001f || ey > .005f || er > .005f)
                            {
                                failures.Add(new { tool = tool.name, placement = placement.name, index = i, source = Values(p), joints = Values(j), ef, el, ew, ey, er });
                                groupFailed++;
                            }
                        }
                    }
                    groups.Add(new { tool = tool.name, placement = placement.name, targets = poses.Count, candidates = groupCandidates, failures = groupFailed });
                }
            }
            // Independent model comparison must detect a mount change omitted from the inputs.
            root.transform.SetPositionAndRotation(rootPosition,rootRotation);
            gripper.localPosition = hg + Vector3.down * .02f;
            gripper.localRotation = hgr;
            var negativeGoal = new Vector3(.4f,-.2f,-.3f);
            var negative = IK.Solve(negativeGoal,0f,Geometry(new Vector3(0f,-.034177f,0f),0f),Limits);
            Require(negative.Success, "Negative control has no IK solution");
            var nj = negative.Solutions[0].Joints;
            a1.localRotation = h1 * Quaternion.AngleAxis(nj.Q1Degrees,Vector3.up);
            a2.localRotation = h2 * Quaternion.AngleAxis(nj.Q2Degrees,Vector3.up);
            a3.localPosition = h3 + Vector3.down * nj.D3Metres;
            a4.localRotation = h4 * Quaternion.AngleAxis(nj.Q4Degrees,Vector3.up);
            float mountMismatch = Vector3.Distance(frame.InverseTransformPoint(tcp.position),negativeGoal);
            bool unchanged = SourcePaths.All(path => hashes[path] == Hash(path));
            bool passed = failures.Count == 0 && unchanged && Mathf.Abs(mountMismatch-.02f) < .00001f;
            var report = new { passed, utc = DateTime.UtcNow.ToString("O"), seed = Seed, distinctJointPoses = poses.Count,
                placements = placements.Length, toolVariants = tools.Length, staticTargets = targets, candidatesChecked = candidates,
                failures = failures.Count, maxFkPositionErrorMetres = maxFk, maxActualLocalErrorMetres = maxLocal,
                maxActualWorldErrorMetres = maxWorld, maxActualYawErrorDegrees = maxYaw, maxActualWorldRotationErrorDegrees = maxRotation,
                uncalibratedMountErrorMetres = mountMismatch, sourceAssetsUnchanged = unchanged, sourceHashes = hashes,
                positionToleranceMetres = .00001f, angleToleranceDegrees = .005f,
                scope = "Static isolated model validation of every returned IK candidate; not motor cycles or collision safety",
                jointPoses = poses.Select(Values).ToArray(), groups, failureDetails = failures };
            Write("full-static-verification.json",report);
            Require(passed, "Full static IK verification failed; inspect Logs/LW06/full-static-verification.json");
            return new { passed, report.distinctJointPoses, report.staticTargets, report.candidatesChecked, report.failures,
                report.maxFkPositionErrorMetres, report.maxActualLocalErrorMetres, report.maxActualWorldErrorMetres,
                report.maxActualYawErrorDegrees, report.maxActualWorldRotationErrorDegrees, report.uncalibratedMountErrorMetres, unchanged };
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
    }

    public static Task<object> Play() => PlayInto("Logs/LW06");
    public static Task<object> ContinuityRegression() => PlayInto("Logs/LW07");
    public static Task<object> PathRegression() => PlayInto("Logs/LW07Path");

    private static async Task<object> PlayInto(string reportDirectory)
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused && Time.timeScale > 0f, "Unpaused Play Mode required");
        var lab = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        Require(lab != null && lab.IsReady, "RobotLab references required");
        var hashes = SourcePaths.ToDictionary(path => path, Hash);
        var settings = new SerializedObject(lab);
        var geometry = new ScaraForwardKinematics.Geometry(
            settings.FindProperty("fkLink1Metres").floatValue,settings.FindProperty("fkLink2Metres").floatValue,
            settings.FindProperty("fkHeight12Metres").floatValue,settings.FindProperty("fkHeight23Metres").floatValue,
            settings.FindProperty("fkToolOffsetMetres").vector3Value,settings.FindProperty("fkHomeToolYawDegrees").floatValue);
        var token = lab.destroyCancellationToken;
        float originalScale = Time.timeScale, originalStep = Time.fixedDeltaTime;
        var rows = new List<object>();
        int movingGoals = 0, rejected = 0, movingSamples = 0;
        float maxPosition = 0f, maxYaw = 0f, maxMovingFk = 0f;
        var watch = Stopwatch.StartNew();
        try
        {
            Time.timeScale = 10f;
            lab.HomeAll();
            await Arrive(lab,token);
            Require(lab.TryReadFkDiagnostics(out var initial,out _) && initial.PositionErrorMetres < .00001f, "Scene geometry does not match its model");
            var poses = Poses();
            for (int i = 0; i < poses.Count; i++)
            {
                var p = poses[i];
                var goal = ScaraForwardKinematics.Calculate(p.x,p.y,p.z,p.w,geometry);
                var mode = i % 3 == 0 ? ScaraJointLab.IkSelection.Nearest : p.y < 0f ? ScaraJointLab.IkSelection.NegativeElbow : ScaraJointLab.IkSelection.PositiveElbow;
                Require(lab.TryMoveTcp(goal.PositionMetres,goal.YawDegrees,mode,out var selected,out var error), "Goal " + i + " rejected: " + error);
                if (Moving(lab)) movingGoals++;
                if (i == 1 || i == 33 || i == 66 || i == 99)
                {
                    var targets = Targets(lab); var current = Current(lab); var flags = Flags(lab);
                    var invalid = i == 1 ? new Vector3(.8f,-.2f,0f) : i == 33 ? new Vector3(-.7f,-.2f,0f) : i == 66 ? new Vector3(float.NaN,0f,0f) : goal.PositionMetres;
                    var invalidMode = i == 99 ? (ScaraJointLab.IkSelection)99 : ScaraJointLab.IkSelection.Nearest;
                    Require(!lab.TryMoveTcp(invalid,i == 33 ? 180f : 0f,invalidMode,out _,out var failure) && !string.IsNullOrEmpty(failure), "Invalid request accepted");
                    Require(targets.Equals(Targets(lab)) && current.Equals(Current(lab)) && flags.SequenceEqual(Flags(lab)), "Invalid request partially changed commands or stopped motion");
                    rejected++;
                }
                await Arrive(lab,token,() => {
                    Require(lab.TryReadFkDiagnostics(out var sample,out _) && sample.PositionErrorMetres <= .00001f, "Moving FK/model mismatch");
                    maxMovingFk = Mathf.Max(maxMovingFk,sample.PositionErrorMetres);
                    movingSamples++;
                });
                float ep = Vector3.Distance(lab.ActualTcpPosition,goal.PositionMetres);
                float ey = Mathf.Abs(Mathf.DeltaAngle(lab.ActualTcpYawDegrees,goal.YawDegrees));
                Require(ep <= .002f && ey <= .5f, "Goal " + i + " failed actual arrival tolerances");
                maxPosition = Mathf.Max(maxPosition,ep); maxYaw = Mathf.Max(maxYaw,ey);
                rows.Add(new { index = i, sourceJoints = Values(p), targetMetres = Values(goal.PositionMetres), targetYawDegrees = goal.YawDegrees,
                    selectedJoints = Values(selected.Joints), selection = mode.ToString(), selected.ElbowSign, selected.IsSingular,
                    actualMetres = Values(lab.ActualTcpPosition), positionErrorMetres = ep, yawErrorDegrees = ey });
            }
            lab.HomeAll();
            await Arrive(lab,token);
            bool unchanged = SourcePaths.All(path => hashes[path] == Hash(path));
            Require(unchanged && Time.fixedDeltaTime == originalStep, "Source files or fixed timestep changed");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("O"), seed = Seed, actualTargetsVerified = rows.Count,
                movesWithJointMotion = movingGoals, rejectedRequestsPreserved = rejected, movingSamples,
                maxActualPositionErrorMetres = maxPosition, maxActualYawErrorDegrees = maxYaw, maxMovingFkErrorMetres = maxMovingFk,
                simulatedTimeScale = 10f, fixedDeltaTime = originalStep, wallSeconds = watch.Elapsed.TotalSeconds,
                positionToleranceMetres = .002f, angleToleranceDegrees = .5f, sourceAssetsUnchanged = unchanged,
                directButtonClicks = 0, scope = "100 API-driven arrivals via actual FixedUpdate axes; accelerated simulation, no collision or real robot accuracy claim", rows };
            Write("full-play-verification.json",report,reportDirectory);
            return new { report.passed, report.actualTargetsVerified, report.movesWithJointMotion, report.rejectedRequestsPreserved,
                report.maxActualPositionErrorMetres, report.maxActualYawErrorDegrees, report.maxMovingFkErrorMetres,
                report.movingSamples, report.wallSeconds, unchanged };
        }
        finally
        {
            Time.timeScale = originalScale;
            if (lab != null) lab.StopAll();
        }
    }

    private static List<Vector4> Poses()
    {
        var poses = new List<Vector4> {
            new Vector4(0f,0f,0f,0f),new Vector4(90f,0f,0f,0f),new Vector4(-90f,0f,0f,0f),
            new Vector4(0f,90f,0f,0f),new Vector4(90f,-90f,0f,0f),new Vector4(0f,0f,.05f,0f),
            new Vector4(0f,0f,0f,90f),new Vector4(45f,0f,.05f,-45f),
            new Vector4(-170f,-150f,0f,-180f),new Vector4(170f,150f,.15f,180f),
            new Vector4(0f,0f,.15f,0f),new Vector4(0f,.0001f,.075f,180f),
            new Vector4(170f,-150f,.15f,-180f),new Vector4(-170f,150f,0f,180f),
            new Vector4(0f,-150f,.075f,-180f),new Vector4(0f,150f,.075f,180f)
        };
        var random = new System.Random(Seed);
        while (poses.Count < 100) poses.Add(new Vector4((float)(random.NextDouble()*340-170),
            (float)(random.NextDouble()*300-150),(float)(random.NextDouble()*.15),(float)(random.NextDouble()*360-180)));
        return poses;
    }

    private static ScaraForwardKinematics.Geometry Geometry(Vector3 offset,float yaw) => new ScaraForwardKinematics.Geometry(.325f,.375f,.016783f,-.097783f,offset,yaw);
    private static bool InRange(IK.JointValues j) => j.Q1Degrees >= -170f && j.Q1Degrees <= 170f && j.Q2Degrees >= -150f && j.Q2Degrees <= 150f && j.D3Metres >= 0f && j.D3Metres <= .15f && j.Q4Degrees >= -180f && j.Q4Degrees <= 180f;
    private static Vector4 Targets(ScaraJointLab l) => new Vector4(l.Axis1.TargetPosition,l.Axis2.TargetPosition,l.Axis3.TargetPosition,l.Axis4.TargetPosition);
    private static Vector4 Current(ScaraJointLab l) => new Vector4(l.Axis1.CurrentPosition,l.Axis2.CurrentPosition,l.Axis3.CurrentPosition,l.Axis4.CurrentPosition);
    private static bool[] Flags(ScaraJointLab l) => new[] { l.Axis1.IsMoving,l.Axis2.IsMoving,l.Axis3.IsMoving,l.Axis4.IsMoving };
    private static bool Moving(ScaraJointLab l) => Flags(l).Any(flag => flag);
    private static async Task Arrive(ScaraJointLab lab,CancellationToken token,Action sample = null)
    {
        var watch = Stopwatch.StartNew();
        while (Moving(lab) || !lab.Axis1.IsAtTarget || !lab.Axis2.IsAtTarget || !lab.Axis3.IsAtTarget || !lab.Axis4.IsAtTarget)
        {
            token.ThrowIfCancellationRequested();
            Require(!EditorApplication.isPaused, "Verification interrupted by user pause");
            if (watch.Elapsed.TotalSeconds > 12) throw new TimeoutException("Actual axis arrival timed out");
            sample?.Invoke();
            await Task.Delay(15,token);
        }
    }
    private static float RotationError(Quaternion expected,Quaternion actual)
    {
        var delta = Quaternion.Inverse(expected)*actual;
        return 2f*Mathf.Atan2(new Vector3(delta.x,delta.y,delta.z).magnitude,Mathf.Abs(delta.w))*Mathf.Rad2Deg;
    }
    private static float[] Values(Vector3 v) => new[] { v.x,v.y,v.z };
    private static float[] Values(Vector4 v) => new[] { v.x,v.y,v.z,v.w };
    private static float[] Values(IK.JointValues j) => new[] { j.Q1Degrees,j.Q2Degrees,j.D3Metres,j.Q4Degrees };
    private static void Write(string name,object value,string directory = "Logs/LW06")
    {
        Directory.CreateDirectory(directory);
        File.WriteAllText(Path.Combine(directory,name),JsonConvert.SerializeObject(value,Formatting.Indented));
    }
    private static void Require(bool condition,string message) { if (!condition) throw new InvalidOperationException(message); }
    private static string Hash(string path)
    {
        using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", "");
    }
}
