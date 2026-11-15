using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using Home = LineWatch.FanucKinematicGeometry.JointHome;

public static class VerifyFanucForwardKinematics
{
    private const string PrefabPath = "Assets/03.Prefabs/FanucRobot_Work.prefab";

    public static object References()
    {
        Require(!EditorApplication.isPlaying, "Run fixed references in Edit Mode.");
        var paths = new[] { PrefabPath, "Assets/01.Scenes/RobotLab.unity",
            "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };
        var hashes = paths.ToDictionary(p => p, Hash);
        var root = PrefabUtility.LoadPrefabContents(PrefabPath);
        try
        {
            var lab = root.GetComponent<FanucJointLab>();
            Require(lab != null && lab.Geometry != null && lab.RobotFrame == root.transform, "Missing saved geometry/frame.");
            var homes = lab.Joints.Select(j => j.transform.localRotation).ToArray();
            var samples = new List<float[]> { new float[6] };
            for (int i = 0; i < 6; i++)
                foreach (float degrees in new[] { 30f, -30f })
                {
                    var q = new float[6]; q[i] = degrees; samples.Add(q);
                }
            samples.Add(new[] { 25f, -30f, 20f, -35f, 15f, 40f });
            samples.Add(new[] { -40f, 15f, -25f, 30f, -35f, -20f });
            var cases = samples.Select((q,index) =>
            {
                var input = (float[])q.Clone();
                for (int i = 0; i < 6; i++)
                    lab.Joints[i].transform.localRotation = homes[i] * Quaternion.AngleAxis(q[i], lab.Joints[i].LocalRotationAxis);
                var before = lab.Joints.Select(j => j.transform.localRotation).ToArray();
                var fk = FanucForwardKinematics.Calculate(q, lab.Geometry);
                var reference = Reference(q);
                var actualPosition = lab.RobotFrame.InverseTransformPoint(lab.Tcp.position);
                var actualRotation = Quaternion.Inverse(lab.RobotFrame.rotation) * lab.Tcp.rotation;
                float referencePositionError = Vector3.Distance(fk.PositionMetres, reference.position);
                float modelPositionError = Vector3.Distance(fk.PositionMetres, actualPosition);
                float referenceRotationError = Angle(fk.Rotation, reference.rotation);
                float modelRotationError = Angle(fk.Rotation, actualRotation);
                Require(referencePositionError < 0.000002f && modelPositionError < 0.000002f &&
                    referenceRotationError < 0.001f && modelRotationError < 0.001f, "FK disagreement at fixed reference " + index);
                Require(q.SequenceEqual(input) && lab.Joints.Select((j,i) => j.transform.localRotation == before[i]).All(v => v),
                    "Calculation mutated input or model.");
                return new { name = index == 0 ? "home" : index <= 12 ? "A" + ((index+1)/2) + (index%2 == 1 ? "+30" : "-30") : "combined-" + (index-12),
                    jointDegrees = q, calculatedPositionMetres = V(fk.PositionMetres), calculatedRotationXyzw = Q(fk.Rotation),
                    referencePositionMetres = V(reference.position), referenceRotationXyzw = Q(reference.rotation),
                    actualPositionMetres = V(actualPosition), actualRotationXyzw = Q(actualRotation),
                    referencePositionErrorMetres = referencePositionError, modelPositionErrorMetres = modelPositionError,
                    referenceRotationErrorDegrees = referenceRotationError, modelRotationErrorDegrees = modelRotationError };
            }).ToArray();
            var functionChecks = FunctionCases(lab.Geometry);
            Require(hashes.All(p => Hash(p.Key) == p.Value), "A source asset changed during isolated reference tests.");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"),
                scope = "LW-07B first process: pure six-axis FK, 15 fixed references and isolated prefab; no runtime diagnostics/100-pose campaign/IK",
                referenceMethod = "Independent RobotFrame-space home screw axes and pivots (POE); not runtime home-link recursion",
                caseCount = cases.Length, maxReferencePositionErrorMetres = cases.Max(c => c.referencePositionErrorMetres),
                maxModelPositionErrorMetres = cases.Max(c => c.modelPositionErrorMetres),
                maxReferenceRotationErrorDegrees = cases.Max(c => c.referenceRotationErrorDegrees),
                maxModelRotationErrorDegrees = cases.Max(c => c.modelRotationErrorDegrees),
                functionChecks, sourceHashes = hashes, sourceAssetsUnchanged = true, cases };
            Directory.CreateDirectory("Logs/LW07B");
            File.WriteAllText("Logs/LW07B/reference-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return new { report.passed, report.caseCount, report.maxReferencePositionErrorMetres,
                report.maxModelPositionErrorMetres, report.maxReferenceRotationErrorDegrees,
                report.maxModelRotationErrorDegrees, report.functionChecks, report.sourceAssetsUnchanged };
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
    }

    // Independent home pivots/axes in RobotFrame. Apply fixed-space rotations in reverse
    // order to a home point (POE), rather than translating through joint-local frames.
    private static (Vector3 position, Quaternion rotation) Reference(float[] q)
    {
        var pivots = new[] { Vector3.zero, new Vector3(0,.245f,0), new Vector3(0,.955f,0),
            new Vector3(0,.955f,-.15f), new Vector3(0,.955f,-.54f), new Vector3(-.15f,.955f,-.64f) };
        var axes = new[] { Vector3.down, Vector3.right, Vector3.right, Vector3.back, Vector3.right, Vector3.back };
        var position = new Vector3(-.15f,.955f,-.7f);
        var rotation = new Quaternion(-Mathf.Sqrt(.5f),-Mathf.Sqrt(.5f),0,0);
        for (int i = 5; i >= 0; i--)
        {
            var turn = Quaternion.AngleAxis(q[i], axes[i]);
            position = pivots[i] + turn * (position - pivots[i]);
            rotation = turn * rotation;
        }
        return (position, Quaternion.Normalize(rotation));
    }

    private static object FunctionCases(FanucKinematicGeometry geometry)
    {
        var zeros = new float[6];
        var beyondMotorRange = new[] { 0f, 0f, 0f, 0f, 0f, 90f };
        var unclamped = FanucForwardKinematics.Calculate(beyondMotorRange, geometry);
        var expected = Reference(beyondMotorRange);
        Require(Vector3.Distance(unclamped.PositionMetres, expected.position) < .000002f &&
            Angle(unclamped.Rotation, expected.rotation) < .001f, "Pure FK clamped to a motor range.");
        var scaled = geometry.Joints.Select(h => new Home(h.PositionMetres, h.Rotation, h.LocalRotationAxis * 3f)).ToArray();
        var equivalent = new FanucKinematicGeometry(scaled, geometry.ToolPositionMetres, geometry.ToolRotation);
        var normalized = FanucForwardKinematics.Calculate(beyondMotorRange, equivalent);
        Require(Vector3.Distance(unclamped.PositionMetres, normalized.PositionMetres) < .000002f &&
            Angle(unclamped.Rotation, normalized.Rotation) < .001f, "Axis direction normalization changed the pose.");
        // Noncommuting home/tool rotations, with a tool offset in all three directions.
        var customHomes = Enumerable.Range(0,6).Select(_ => new Home(Vector3.zero, Quaternion.identity, Vector3.up)).ToArray();
        customHomes[0] = new Home(new Vector3(.1f,.2f,.3f), Quaternion.AngleAxis(90,Vector3.right), Vector3.up);
        customHomes[1] = new Home(new Vector3(0,.4f,0), Quaternion.AngleAxis(90,Vector3.forward), Vector3.right);
        var custom = new FanucKinematicGeometry(customHomes, new Vector3(.1f,.2f,.3f), Quaternion.AngleAxis(90,Vector3.up));
        var customPose = FanucForwardKinematics.Calculate(zeros, custom);
        // Rx90 * Rz90 maps (x,y,z) to (-y,-z,x).
        Require(Vector3.Distance(customPose.PositionMetres, new Vector3(-.1f,-.1f,.8f)) < .000002f &&
            Vector3.Distance(customPose.Rotation * Vector3.forward, Vector3.forward) < .000002f &&
            Vector3.Distance(customPose.Rotation * Vector3.up, Vector3.left) < .000002f, "Noncommuting home/tool transform order is wrong.");

        int rejected = 0;
        var invalid = new List<Action> {
            () => FanucForwardKinematics.Calculate(null, geometry),
            () => FanucForwardKinematics.Calculate(new float[5], geometry),
            () => FanucForwardKinematics.Calculate(new float[7], geometry),
            () => FanucForwardKinematics.Calculate(zeros, null),
            () => FanucForwardKinematics.Calculate(zeros, new FanucKinematicGeometry()),
            () => FanucForwardKinematics.Calculate(zeros, new FanucKinematicGeometry(null, Vector3.zero, Quaternion.identity)),
            () => FanucForwardKinematics.Calculate(zeros, new FanucKinematicGeometry(new Home[5], Vector3.zero, Quaternion.identity)),
            () => FanucForwardKinematics.Calculate(zeros, new FanucKinematicGeometry(geometry.Joints.ToArray(), new Vector3(float.NaN,0,0), Quaternion.identity)),
            () => FanucForwardKinematics.Calculate(zeros, new FanucKinematicGeometry(geometry.Joints.ToArray(), Vector3.zero, default)) };
        for (int i = 0; i < 6; i++)
            foreach (float value in new[] { float.NaN, float.PositiveInfinity, float.NegativeInfinity })
            {
                var input = new float[6]; input[i] = value;
                invalid.Add(() => FanucForwardKinematics.Calculate(input, geometry));
            }
        foreach (var badHome in new Home[] { null, new Home(new Vector3(float.NaN,0,0), Quaternion.identity, Vector3.up),
            new Home(Vector3.zero, default, Vector3.up), new Home(Vector3.zero,new Quaternion(0,0,0,2),Vector3.up),
            new Home(Vector3.zero,new Quaternion(float.PositiveInfinity,0,0,1),Vector3.up),
            new Home(Vector3.zero,Quaternion.identity,Vector3.zero),
            new Home(Vector3.zero,Quaternion.identity,new Vector3(0,float.NaN,0)) })
        {
            var data = geometry.Joints.ToArray(); data[2] = badHome;
            invalid.Add(() => FanucForwardKinematics.Calculate(zeros, new FanucKinematicGeometry(data, Vector3.zero, Quaternion.identity)));
        }
        foreach (var action in invalid)
        {
            try { action(); }
            catch (ArgumentException) { rejected++; continue; }
            throw new InvalidOperationException("Invalid FK input was accepted.");
        }
        return new { invalidInputsRejected = rejected, unclampedMotorRangePassed = true,
            axisNormalizationPassed = true, noncommutingHomeAndToolPassed = true };
    }

    private static float Angle(Quaternion a, Quaternion b)
    {
        var delta = Quaternion.Inverse(a) * b;
        return (float)(2 * Math.Atan2(Math.Sqrt((double)delta.x*delta.x + (double)delta.y*delta.y +
            (double)delta.z*delta.z), Math.Abs(delta.w)) * 180 / Math.PI);
    }
    private static float[] V(Vector3 value) => new[] { value.x, value.y, value.z };
    private static float[] Q(Quaternion value) => new[] { value.x, value.y, value.z, value.w };
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
