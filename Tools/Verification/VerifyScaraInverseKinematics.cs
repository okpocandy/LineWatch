using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using IK = LineWatch.ScaraInverseKinematics;

public static class VerifyScaraInverseKinematics
{
    public static object Run()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Run pure IK verification in Edit Mode.");
        string[] assets = { "Assets/01.Scenes/RobotLab.unity", "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };
        string[] before = assets.Select(Hash).ToArray();
        var geometry = new ScaraForwardKinematics.Geometry(.325f, .375f, .016783f, -.097783f, new Vector3(0f, -.034177f, 0f));
        var limits = new IK.JointLimits(new IK.JointValues(-170f, -150f, 0f, -180f), new IK.JointValues(170f, 150f, .15f, 180f));
        var rows = new List<object>();
        int successes = 0, rejected = 0, candidateCount = 0;
        float maxPosition = 0f, maxYaw = 0f;

        IK.Result Accept(string name, Vector3 target, float yaw, ScaraForwardKinematics.Geometry g, IK.JointLimits bounds, int? count = null)
        {
            var result = IK.Solve(target, yaw, g, bounds);
            Require(result.Success && result.Solutions.Count > 0, name + ": " + result.Failure);
            if (count.HasValue) Require(result.Solutions.Count == count.Value, name + ": unexpected solution count " + result.Solutions.Count);
            foreach (var solution in result.Solutions)
            {
                var j = solution.Joints;
                Require(InRange(j, bounds), name + ": joint outside bounds");
                var fk = ScaraForwardKinematics.Calculate(j.Q1Degrees, j.Q2Degrees, j.D3Metres, j.Q4Degrees, g);
                float positionError = Vector3.Distance(target, fk.PositionMetres);
                float yawError = Mathf.Abs(Mathf.DeltaAngle(yaw, fk.YawDegrees));
                Require(positionError <= .000002f && yawError <= .001f, name + ": FK residual");
                maxPosition = Mathf.Max(maxPosition, positionError);
                maxYaw = Mathf.Max(maxYaw, yawError);
            }
            successes++;
            candidateCount += result.Solutions.Count;
            rows.Add(new { name, passed = true, targetMetres = new[] { target.x, target.y, target.z }, yaw,
                solutions = result.Solutions.Select(s => new { joints = Values(s.Joints), s.ElbowSign, s.IsSingular }).ToArray() });
            return result;
        }

        void Reject(string name, Vector3 target, float yaw, ScaraForwardKinematics.Geometry g, IK.JointLimits bounds, IK.FailureReason reason)
        {
            var result = IK.Solve(target, yaw, g, bounds);
            Require(!result.Success && result.Failure == reason && result.Solutions.Count == 0, name + ": " + result.Failure);
            rejected++;
            rows.Add(new { name, passed = true, failure = result.Failure.ToString(), candidates = result.Solutions.Count });
        }

        void RoundTrip(string name, IK.JointValues expected, ScaraForwardKinematics.Geometry g, IK.JointLimits bounds)
        {
            var target = ScaraForwardKinematics.Calculate(expected.Q1Degrees, expected.Q2Degrees, expected.D3Metres, expected.Q4Degrees, g);
            var result = Accept(name, target.PositionMetres, target.YawDegrees, g, bounds);
            Require(result.Solutions.Any(s => Close(s.Joints, expected)), name + ": original joint solution missing");
        }

        var home = Accept("Home merged branch", new Vector3(.7f, -.115177f, 0f), 0f, geometry, limits, 1);
        Require(home.Solutions[0].IsSingular && home.Solutions[0].ElbowSign == 0, "Home singular flag");
        var negativeZ = Accept("Two elbow branches", new Vector3(.325f, -.165177f, -.375f), 0f, geometry, limits, 2);
        float a = (float)(2.0 * Math.Atan2(.375, .325) * 180.0 / Math.PI);
        Require(Close(negativeZ.Solutions[0].Joints, new IK.JointValues(0f, 90f, .05f, -90f)) &&
            Close(negativeZ.Solutions[1].Joints, new IK.JointValues(a, -90f, .05f, 90f-a)), "Hand-derived solutions");
        Require(negativeZ.Solutions[0].ElbowSign == 1 && negativeZ.Solutions[1].ElbowSign == -1, "Branch ordering");
        Accept("Positive Z sign", new Vector3(.325f, -.115177f, .375f), 0f, geometry, limits, 2);
        Accept("Rotated target", new Vector3(.375f, -.115177f, -.325f), 0f, geometry, limits, 2);
        Accept("Yaw 180", new Vector3(.325f, -.165177f, -.375f), 180f, geometry, limits, 2);
        Accept("Yaw 540 equivalent", new Vector3(.325f, -.165177f, -.375f), 540f, geometry, limits, 2);
        var wristEnds = Accept("Distinct +180 and -180 wrist endpoints", new Vector3(.7f, -.115177f, 0f), 180f, geometry, limits, 2);
        Require(wristEnds.Solutions.Any(s => s.Joints.Q4Degrees == -180f) && wristEnds.Solutions.Any(s => s.Joints.Q4Degrees == 180f), "Wrist endpoint enumeration");
        var positiveOnly = new IK.JointLimits(new IK.JointValues(-170f, 0f, 0f, -180f), limits.Maximum);
        Accept("Filter negative elbow by joint limits", new Vector3(.325f, -.165177f, -.375f), 0f, geometry, positiveOnly, 1);
        var lockedHome = new IK.JointLimits(new IK.JointValues(0f, 0f, 0f, 0f), new IK.JointValues(0f, 0f, 0f, 0f));
        Accept("Locked home axes", new Vector3(.7f, -.115177f, 0f), 0f, geometry, lockedHome, 1);
        RoundTrip("Joint positive endpoints", new IK.JointValues(170f, 150f, .15f, 180f), geometry, limits);
        RoundTrip("Joint negative endpoints", new IK.JointValues(-170f, -150f, 0f, -180f), geometry, limits);
        RoundTrip("Near straight remains two branches", new IK.JointValues(20f, .5f, .04f, -20f), geometry, limits);
        var nearStraight = ScaraForwardKinematics.Calculate(20f, .5f, .04f, -20f, geometry);
        Require(IK.Solve(nearStraight.PositionMetres, nearStraight.YawDegrees, geometry, limits).Solutions.Count == 2, "Near straight branches must not merge");
        var tool = new ScaraForwardKinematics.Geometry(.325f, .375f, .016783f, -.097783f, new Vector3(.02f, -.054177f, .01f), 35f);
        var handTool = Accept("Hand-derived eccentric tool and home yaw", new Vector3(.345f, -.185177f, -.365f), 35f, tool, limits, 2);
        Require(handTool.Solutions.Any(s => Close(s.Joints, new IK.JointValues(0f, 90f, .05f, -90f))), "Hand-derived tool solution");
        RoundTrip("Horizontal tool offset with home yaw", new IK.JointValues(20f, -50f, .07f, 40f), tool, limits);
        var tool2 = new ScaraForwardKinematics.Geometry(.325f, .375f, .016783f, -.097783f, new Vector3(-.03f, -.02f, .015f), -73f);
        RoundTrip("Opposite tool offset and home yaw", new IK.JointValues(-45f, 80f, .03f, -70f), tool2, limits);
        Accept("Cosine roundoff just outside extension", new Vector3(.7f + 1e-7f, -.115177f, 0f), 0f, geometry, limits, 1);
        var fullTurn = new IK.JointLimits(new IK.JointValues(-180f, -180f, 0f, -180f), new IK.JointValues(180f, 180f, .15f, 180f));
        var folded = Accept("Unequal links fully folded endpoints", new Vector3(.05f, -.115177f, 0f), 0f, geometry, fullTurn);
        Require(folded.Solutions.All(s => s.IsSingular), "Folded singular flag");
        var equal = new ScaraForwardKinematics.Geometry(.5f, .5f, 0f, 0f, Vector3.zero);
        Reject("Equal-link shoulder continuum", Vector3.zero, 0f, equal, fullTurn, IK.FailureReason.Indeterminate);
        Reject("Outer reach", new Vector3(.8f, -.115177f, 0f), 0f, geometry, limits, IK.FailureReason.Unreachable);
        Reject("Inner reach", new Vector3(.01f, -.115177f, 0f), 0f, geometry, limits, IK.FailureReason.Unreachable);
        Reject("Outside cosine roundoff tolerance", new Vector3(.7f + 1e-6f, -.115177f, 0f), 0f, geometry, limits, IK.FailureReason.Unreachable);
        Reject("Shoulder outside range", new Vector3(-.7f, -.115177f, 0f), 180f, geometry, limits, IK.FailureReason.JointLimit);
        Reject("Elbow outside range", new Vector3(.05f, -.115177f, 0f), 0f, geometry, limits, IK.FailureReason.JointLimit);
        var restrictedWrist = new IK.JointLimits(new IK.JointValues(-170f, -150f, 0f, -45f), new IK.JointValues(170f, 150f, .15f, 45f));
        Reject("Position reachable but orientation excludes both branches", new Vector3(.325f, -.115177f, -.375f), 180f, geometry, restrictedWrist, IK.FailureReason.JointLimit);
        Reject("Slide above home", new Vector3(.325f, -.105177f, -.375f), 0f, geometry, limits, IK.FailureReason.JointLimit);
        Reject("Slide below travel", new Vector3(.325f, -.275177f, -.375f), 0f, geometry, limits, IK.FailureReason.JointLimit);
        Reject("NaN position", new Vector3(float.NaN, 0f, 0f), 0f, geometry, limits, IK.FailureReason.InvalidInput);
        Reject("Infinite position", new Vector3(0f, 0f, float.PositiveInfinity), 0f, geometry, limits, IK.FailureReason.InvalidInput);
        Reject("NaN yaw", Vector3.zero, float.NaN, geometry, limits, IK.FailureReason.InvalidInput);
        Reject("Infinite yaw", Vector3.zero, float.NegativeInfinity, geometry, limits, IK.FailureReason.InvalidInput);
        Reject("Zero default links", Vector3.zero, 0f, default, limits, IK.FailureReason.InvalidInput);
        Reject("NaN tool", Vector3.zero, 0f, new ScaraForwardKinematics.Geometry(.325f, .375f, 0f, 0f, new Vector3(float.NaN, 0f, 0f)), limits, IK.FailureReason.InvalidInput);
        Reject("Negative link", Vector3.zero, 0f, new ScaraForwardKinematics.Geometry(-.325f, .375f, 0f, 0f, Vector3.zero), limits, IK.FailureReason.InvalidInput);
        Reject("Reversed limits", Vector3.zero, 0f, geometry, new IK.JointLimits(limits.Maximum, limits.Minimum), IK.FailureReason.InvalidInput);
        Reject("Infinite limit", Vector3.zero, 0f, geometry, new IK.JointLimits(new IK.JointValues(float.NegativeInfinity, -150f, 0f, -180f), limits.Maximum), IK.FailureReason.InvalidInput);
        Reject("Unsupported multi-turn limits", Vector3.zero, 0f, geometry, new IK.JointLimits(new IK.JointValues(-200f, -150f, 0f, -180f), limits.Maximum), IK.FailureReason.InvalidInput);
        // Valid doubles can still exceed the existing float FK's arithmetic range.
        var extreme = new ScaraForwardKinematics.Geometry(1e20f, 1e20f, 0f, 0f, Vector3.zero);
        Reject("Float FK residual overflow", new Vector3(1e20f, 0f, 0f), 0f, extreme, limits, IK.FailureReason.NumericalError);

        bool assetsUnchanged = assets.Select(Hash).SequenceEqual(before);
        Require(assetsUnchanged, "Existing scene, prefab or settings changed");
        var report = new { passed = true, utc = DateTime.UtcNow.ToString("O"), successfulTargets = successes,
            rejectedTargets = rejected, candidatesChecked = candidateCount, maxPositionErrorMetres = maxPosition,
            maxYawErrorDegrees = maxYaw, sourceAssetsUnchanged = assetsUnchanged,
            scope = "Pure IK/FK function checks; no motor, UI, world placement or 100-target integration verification", rows };
        Directory.CreateDirectory("Logs/LW06");
        File.WriteAllText("Logs/LW06/inverse-kinematics-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;
    }

    private static bool InRange(IK.JointValues j, IK.JointLimits b) =>
        j.Q1Degrees >= b.Minimum.Q1Degrees && j.Q1Degrees <= b.Maximum.Q1Degrees &&
        j.Q2Degrees >= b.Minimum.Q2Degrees && j.Q2Degrees <= b.Maximum.Q2Degrees &&
        j.D3Metres >= b.Minimum.D3Metres && j.D3Metres <= b.Maximum.D3Metres &&
        j.Q4Degrees >= b.Minimum.Q4Degrees && j.Q4Degrees <= b.Maximum.Q4Degrees;
    private static bool Close(IK.JointValues a, IK.JointValues b) =>
        Mathf.Abs(a.Q1Degrees-b.Q1Degrees) < .02f && Mathf.Abs(a.Q2Degrees-b.Q2Degrees) < .02f &&
        Mathf.Abs(a.D3Metres-b.D3Metres) < .000001f && Mathf.Abs(a.Q4Degrees-b.Q4Degrees) < .02f;
    private static float[] Values(IK.JointValues j) => new[] { j.Q1Degrees, j.Q2Degrees, j.D3Metres, j.Q4Degrees };
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
    private static string Hash(string path)
    {
        using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", "");
    }
}
