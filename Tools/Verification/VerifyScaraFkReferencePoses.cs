using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class VerifyScaraFkReferencePoses
{
    public static object Run()
    {
        return Verify(false);
    }

    public static object VerifyCalculation()
    {
        return Verify(true);
    }

    private static object Verify(bool checkCalculation)
    {
        const string prefabPath = "Assets/03.Prefabs/ScaraRobot_Work.prefab";
        string assetHashBefore = Hash(prefabPath);
        var root = PrefabUtility.LoadPrefabContents(prefabPath);
        try
        {
            var frame = root.transform.Find("RobotFrame");
            var axis1 = root.transform.Find("Base/Axis1");
            var axis2 = axis1.Find("Axis2");
            var axis3 = axis2.Find("Axis3");
            var axis4 = axis3.Find("Axis4");
            var tcp = axis4.Find("Gripper/TCP");
            var home1 = axis1.localRotation;
            var home2 = axis2.localRotation;
            var home3 = axis3.localPosition;
            var home4 = axis4.localRotation;
            float diagonal = 0.7f / Mathf.Sqrt(2f);
            var geometry = new ScaraForwardKinematics.Geometry(0.325f, 0.375f, 0.016783f, -0.097783f,
                new Vector3(0f, -0.034177f, 0f));
            // Hand-computable cases shared by the layout and runtime calculation checks.
            var cases = new[] {
                new { name = "Home", q1 = 0f, q2 = 0f, d3 = 0f, q4 = 0f, expected = new Vector3(0.7f,-0.115177f,0f), yaw = 0f },
                new { name = "Shoulder +90", q1 = 90f, q2 = 0f, d3 = 0f, q4 = 0f, expected = new Vector3(0f,-0.115177f,-0.7f), yaw = 90f },
                new { name = "Shoulder -90", q1 = -90f, q2 = 0f, d3 = 0f, q4 = 0f, expected = new Vector3(0f,-0.115177f,0.7f), yaw = -90f },
                new { name = "Elbow +90", q1 = 0f, q2 = 90f, d3 = 0f, q4 = 0f, expected = new Vector3(0.325f,-0.115177f,-0.375f), yaw = 90f },
                new { name = "Shoulder +90 Elbow -90", q1 = 90f, q2 = -90f, d3 = 0f, q4 = 0f, expected = new Vector3(0.375f,-0.115177f,-0.325f), yaw = 0f },
                new { name = "Slide 50mm down", q1 = 0f, q2 = 0f, d3 = 0.05f, q4 = 0f, expected = new Vector3(0.7f,-0.165177f,0f), yaw = 0f },
                new { name = "Wrist +90", q1 = 0f, q2 = 0f, d3 = 0f, q4 = 90f, expected = new Vector3(0.7f,-0.115177f,0f), yaw = 90f },
                new { name = "Diagonal with wrist compensation", q1 = 45f, q2 = 0f, d3 = 0.05f, q4 = -45f,
                    expected = new Vector3(diagonal,-0.165177f,-diagonal), yaw = 0f }
            };
            var results = cases.Select(pose => {
                axis1.localRotation = home1 * Quaternion.AngleAxis(pose.q1, Vector3.up);
                axis2.localRotation = home2 * Quaternion.AngleAxis(pose.q2, Vector3.up);
                axis3.localPosition = home3 + Vector3.down * pose.d3;
                axis4.localRotation = home4 * Quaternion.AngleAxis(pose.q4, Vector3.up);
                Vector3 actual = frame.InverseTransformPoint(tcp.position);
                float actualYaw = Vector3.SignedAngle(frame.forward, tcp.forward, frame.up);
                float positionError = Vector3.Distance(actual, pose.expected);
                float yawError = Mathf.Abs(Mathf.DeltaAngle(pose.yaw, actualYaw));
                if (positionError > 0.000002f || yawError > 0.001f)
                    throw new InvalidOperationException("Reference pose does not match authored geometry: " + pose.name);
                float? fkReferenceError = null, fkTransformError = null, fkYawError = null;
                if (checkCalculation)
                {
                    var calculated = ScaraForwardKinematics.Calculate(pose.q1, pose.q2, pose.d3, pose.q4, geometry);
                    fkReferenceError = Vector3.Distance(calculated.PositionMetres, pose.expected);
                    fkTransformError = Vector3.Distance(calculated.PositionMetres, actual);
                    fkYawError = Mathf.Abs(Mathf.DeltaAngle(calculated.YawDegrees, actualYaw));
                    if (fkReferenceError > 0.000002f || fkTransformError > 0.000002f || fkYawError > 0.001f)
                        throw new InvalidOperationException("Runtime FK disagrees with the reference or actual TCP: " + pose.name);
                }
                return new { pose.name, pose.q1, pose.q2, pose.d3, pose.q4,
                    expectedMetres = Values(pose.expected), actualMetres = Values(actual),
                    expectedYawDegrees = pose.yaw, actualYawDegrees = actualYaw,
                    positionErrorMetres = positionError, yawErrorDegrees = yawError,
                    fkReferenceErrorMetres = fkReferenceError, fkTransformErrorMetres = fkTransformError, fkYawErrorDegrees = fkYawError };
            }).ToArray();
            object functionChecks = checkCalculation ? VerifyFunctionCases(geometry) : null;
            bool unchanged = assetHashBefore == Hash(prefabPath);
            if (!unchanged) throw new InvalidOperationException("The source prefab changed during read-only reference inspection.");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"),
                scope = checkCalculation ? "LW-05b pure FK function against eight fixed references and isolated model; no runtime panel integration"
                    : "LW-05a fixed reference poses in an isolated prefab copy; no joint actuation",
                sourcePrefabUnchanged = unchanged, caseCount = results.Length,
                maxPositionErrorMetres = results.Max(r => r.positionErrorMetres),
                maxYawErrorDegrees = results.Max(r => r.yawErrorDegrees),
                maxFkReferenceErrorMetres = results.Max(r => r.fkReferenceErrorMetres),
                maxFkTransformErrorMetres = results.Max(r => r.fkTransformErrorMetres),
                maxFkYawErrorDegrees = results.Max(r => r.fkYawErrorDegrees), functionChecks, cases = results };
            Directory.CreateDirectory("Logs/LW05");
            string reportPath = checkCalculation ? "Logs/LW05/forward-kinematics-verification.json" : "Logs/LW05/reference-poses.json";
            File.WriteAllText(reportPath, JsonConvert.SerializeObject(report, Formatting.Indented));
            return new { report.passed, report.caseCount, report.maxPositionErrorMetres, report.maxYawErrorDegrees,
                report.maxFkReferenceErrorMetres, report.maxFkTransformErrorMetres, report.maxFkYawErrorDegrees,
                report.functionChecks, report.sourcePrefabUnchanged, report.scope };
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
    }

    private static object VerifyFunctionCases(ScaraForwardKinematics.Geometry geometry)
    {
        var offsetGeometry = new ScaraForwardKinematics.Geometry(0.325f, 0.375f, 0.016783f, -0.097783f,
            new Vector3(0.02f, -0.034177f, 0f), 45f);
        var offsetHome = ScaraForwardKinematics.Calculate(0f, 0f, 0f, 0f, offsetGeometry);
        var offsetTurn = ScaraForwardKinematics.Calculate(0f, 0f, 0f, 90f, offsetGeometry);
        Require(Vector3.Distance(offsetHome.PositionMetres, new Vector3(0.72f, -0.115177f, 0f)) < 0.000002f &&
            Mathf.Abs(Mathf.DeltaAngle(offsetHome.YawDegrees, 45f)) < 0.001f, "Horizontal tool offset or home yaw is incorrect.");
        Require(Vector3.Distance(offsetTurn.PositionMetres, new Vector3(0.7f, -0.115177f, -0.02f)) < 0.000002f &&
            Mathf.Abs(Mathf.DeltaAngle(offsetTurn.YawDegrees, 135f)) < 0.001f,
            "Tool home yaw was incorrectly added to the translation rotation.");
        var wrapped = ScaraForwardKinematics.Calculate(170f, 30f, 0f, 0f, geometry);
        Require(Mathf.Abs(wrapped.YawDegrees + 160f) < 0.001f, "Yaw did not wrap across 180 degrees.");
        var unclamped = ScaraForwardKinematics.Calculate(0f, 0f, 0.2f, 0f, geometry);
        Require(Mathf.Abs(unclamped.PositionMetres.y + 0.315177f) < 0.000002f,
            "The pure calculation must not clamp a joint value to a motor range.");
        int rejected = 0;
        var invalid = new Action[] {
            () => ScaraForwardKinematics.Calculate(float.NaN, 0f, 0f, 0f, geometry),
            () => ScaraForwardKinematics.Calculate(0f, float.PositiveInfinity, 0f, 0f, geometry),
            () => ScaraForwardKinematics.Calculate(0f, 0f, float.NaN, 0f, geometry),
            () => ScaraForwardKinematics.Calculate(0f, 0f, 0f, float.NegativeInfinity, geometry),
            () => ScaraForwardKinematics.Calculate(0f, 0f, 0f, 0f, default),
            () => ScaraForwardKinematics.Calculate(0f, 0f, 0f, 0f,
                new ScaraForwardKinematics.Geometry(0.325f, 0.375f, 0f, 0f, new Vector3(float.NaN, 0f, 0f)))
        };
        foreach (var action in invalid)
        {
            try { action(); }
            catch (ArgumentException) { rejected++; }
        }
        Require(rejected == invalid.Length, "Non-finite input or invalid geometry was not rejected.");
        return new { horizontalToolAndHomeYawCases = 2, yawWrapPassed = true,
            calculationDoesNotClampMotorRanges = true, invalidInputCasesRejected = rejected };
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }

    private static float[] Values(Vector3 value) => new[] { value.x, value.y, value.z };
    private static string Hash(string path)
    {
        using (var sha = SHA256.Create()) return Convert.ToBase64String(sha.ComputeHash(File.ReadAllBytes(path)));
    }
}
