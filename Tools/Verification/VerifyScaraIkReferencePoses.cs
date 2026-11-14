using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

// Fixed reference solutions for LW-06 planning, not a general-purpose IK solver.
public static class VerifyScaraIkReferencePoses
{
    public static object Run()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Run reference verification in Edit Mode.");
        const string prefabPath = "Assets/03.Prefabs/ScaraRobot_Work.prefab";
        const string scenePath = "Assets/01.Scenes/RobotLab.unity";
        string prefabHash = Hash(prefabPath), sceneHash = Hash(scenePath);
        var root = PrefabUtility.LoadPrefabContents(prefabPath);
        object result;
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
            var geometry = new ScaraForwardKinematics.Geometry(.325f, .375f, .016783f, -.097783f,
                new Vector3(0f, -.034177f, 0f));
            // Reflection of the right-angle elbow triangle gives the second shoulder angle.
            float a = (float)(2.0 * Math.Atan2(.375, .325) * 180.0 / Math.PI);
            var cases = new[] {
                new { name = "Home merged branch", target = new Vector3(.7f,-.115177f,0f), yaw = 0f, q1 = 0f, q2 = 0f, d3 = 0f, q4 = 0f, allowed = true },
                new { name = "Negative Z positive elbow", target = new Vector3(.325f,-.165177f,-.375f), yaw = 0f, q1 = 0f, q2 = 90f, d3 = .05f, q4 = -90f, allowed = true },
                new { name = "Negative Z negative elbow", target = new Vector3(.325f,-.165177f,-.375f), yaw = 0f, q1 = a, q2 = -90f, d3 = .05f, q4 = 90f-a, allowed = true },
                new { name = "Positive Z negative elbow", target = new Vector3(.325f,-.115177f,.375f), yaw = 0f, q1 = 0f, q2 = -90f, d3 = 0f, q4 = 90f, allowed = true },
                new { name = "Positive Z positive elbow", target = new Vector3(.325f,-.115177f,.375f), yaw = 0f, q1 = -a, q2 = 90f, d3 = 0f, q4 = a-90f, allowed = true },
                new { name = "Rotated negative elbow", target = new Vector3(.375f,-.115177f,-.325f), yaw = 0f, q1 = 90f, q2 = -90f, d3 = 0f, q4 = 0f, allowed = true },
                new { name = "Rotated positive elbow", target = new Vector3(.375f,-.115177f,-.325f), yaw = 0f, q1 = 90f-a, q2 = 90f, d3 = 0f, q4 = a-180f, allowed = true },
                new { name = "Yaw boundary positive elbow", target = new Vector3(.325f,-.165177f,-.375f), yaw = 180f, q1 = 0f, q2 = 90f, d3 = .05f, q4 = 90f, allowed = true },
                new { name = "Yaw boundary negative elbow", target = new Vector3(.325f,-.165177f,-.375f), yaw = 180f, q1 = a, q2 = -90f, d3 = .05f, q4 = 270f-a, allowed = true },
                new { name = "Outer radius but shoulder forbidden", target = new Vector3(-.7f,-.115177f,0f), yaw = 180f, q1 = 180f, q2 = 0f, d3 = 0f, q4 = 0f, allowed = false },
                new { name = "Inner radius but elbow forbidden", target = new Vector3(.05f,-.115177f,0f), yaw = 0f, q1 = 180f, q2 = 180f, d3 = 0f, q4 = 0f, allowed = false }
            };
            var rows = cases.Select(pose => {
                axis1.localRotation = home1 * Quaternion.AngleAxis(pose.q1, Vector3.up);
                axis2.localRotation = home2 * Quaternion.AngleAxis(pose.q2, Vector3.up);
                axis3.localPosition = home3 + Vector3.down * pose.d3;
                axis4.localRotation = home4 * Quaternion.AngleAxis(pose.q4, Vector3.up);
                var calculated = ScaraForwardKinematics.Calculate(pose.q1, pose.q2, pose.d3, pose.q4, geometry);
                var actual = frame.InverseTransformPoint(tcp.position);
                float actualYaw = Vector3.SignedAngle(frame.forward, tcp.forward, frame.up);
                float fkError = Vector3.Distance(calculated.PositionMetres, pose.target);
                float actualError = Vector3.Distance(actual, pose.target);
                float yawError = Mathf.Max(Mathf.Abs(Mathf.DeltaAngle(calculated.YawDegrees, pose.yaw)),
                    Mathf.Abs(Mathf.DeltaAngle(actualYaw, pose.yaw)));
                bool allowed = Mathf.Abs(pose.q1) <= 170f && Mathf.Abs(pose.q2) <= 150f &&
                    pose.d3 >= 0f && pose.d3 <= .15f && Mathf.Abs(pose.q4) <= 180f;
                if (fkError > .000002f || actualError > .000002f || yawError > .001f || allowed != pose.allowed)
                    throw new InvalidOperationException("Reference mismatch: " + pose.name);
                return new { pose.name, targetMetres = Values(pose.target), pose.yaw,
                    joints = new[] { pose.q1, pose.q2, pose.d3, pose.q4 }, allowed,
                    fkErrorMetres = fkError, actualErrorMetres = actualError, yawErrorDegrees = yawError };
            }).ToArray();
            result = new { passed = true, utc = DateTime.UtcNow.ToString("O"), fixedReferences = rows.Length,
                allowedReferences = rows.Count(row => row.allowed), excludedReferences = rows.Count(row => !row.allowed),
                reflectedShoulderDegrees = a, maxFkErrorMetres = rows.Max(row => row.fkErrorMetres),
                maxActualErrorMetres = rows.Max(row => row.actualErrorMetres), maxYawErrorDegrees = rows.Max(row => row.yawErrorDegrees),
                sourcePrefabUnchanged = prefabHash == Hash(prefabPath), sourceSceneUnchanged = sceneHash == Hash(scenePath),
                scope = "Fixed IK reference candidates checked with existing FK and isolated model; no IK solver or motor commands", rows };
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
        if (prefabHash != Hash(prefabPath) || sceneHash != Hash(scenePath))
            throw new InvalidOperationException("Source assets changed.");
        Directory.CreateDirectory("Logs/LW06");
        File.WriteAllText("Logs/LW06/reference-solutions.json", JsonConvert.SerializeObject(result, Formatting.Indented));
        return result;
    }

    private static float[] Values(Vector3 value) => new[] { value.x, value.y, value.z };
    private static string Hash(string path)
    {
        using (var sha = SHA256.Create())
            return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", "");
    }
}
