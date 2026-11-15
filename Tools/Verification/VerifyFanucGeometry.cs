using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class VerifyFanucGeometry
{
    private const string PrefabPath = "Assets/03.Prefabs/FanucRobot_Work.prefab";
    private static readonly Vector3[] Axes = { Vector3.down, Vector3.back, Vector3.back,
        Vector3.left, Vector3.back, Vector3.left };
    private static readonly string[] PreservedFiles = { "Assets/01.Scenes/RobotLab.unity",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };

    public static object ConfigureAndVerify()
    {
        Require(!EditorApplication.isPlaying, "Stop Play Mode before recording authored homes.");
        var hashes = PreservedFiles.ToDictionary(p => p, Hash);
        var root = PrefabUtility.LoadPrefabContents(PrefabPath);
        string original;
        try
        {
            var lab = root.GetComponent<FanucJointLab>();
            CheckModel(lab);
            original = ModelFingerprint(lab);
            var serialized = new SerializedObject(lab);
            serialized.FindProperty("robotFrame").objectReferenceValue = root.transform;
            var data = serialized.FindProperty("geometry");
            var joints = data.FindPropertyRelative("joints");
            joints.arraySize = 6;
            for (int i = 0; i < 6; i++)
            {
                var joint = joints.GetArrayElementAtIndex(i);
                joint.FindPropertyRelative("positionMetres").vector3Value = lab.Joints[i].transform.localPosition;
                joint.FindPropertyRelative("rotation").quaternionValue = lab.Joints[i].transform.localRotation;
                joint.FindPropertyRelative("localRotationAxis").vector3Value = lab.Joints[i].LocalRotationAxis;
            }
            data.FindPropertyRelative("toolPositionMetres").vector3Value = lab.Tcp.localPosition;
            data.FindPropertyRelative("toolRotation").quaternionValue = lab.Tcp.localRotation;
            serialized.ApplyModifiedPropertiesWithoutUndo();
            CheckSaved(lab);
            PrefabUtility.SaveAsPrefabAsset(root, PrefabPath, out bool saved);
            Require(saved, "FANUC prefab save failed.");
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }

        root = PrefabUtility.LoadPrefabContents(PrefabPath);
        object report;
        try
        {
            var lab = root.GetComponent<FanucJointLab>();
            CheckSaved(lab);
            Require(original == ModelFingerprint(lab), "Geometry recording changed the authored robot or control settings.");
            var chain = Matrix4x4.identity;
            foreach (var joint in lab.Geometry.Joints)
                chain *= Matrix4x4.TRS(joint.PositionMetres, joint.Rotation, Vector3.one);
            chain *= Matrix4x4.TRS(lab.Geometry.ToolPositionMetres, lab.Geometry.ToolRotation, Vector3.one);
            var actualPosition = lab.RobotFrame.InverseTransformPoint(lab.Tcp.position);
            var actualRotation = Quaternion.Inverse(lab.RobotFrame.rotation) * lab.Tcp.rotation;
            float positionError = Vector3.Distance(chain.GetColumn(3), actualPosition);
            float rotationError = Quaternion.Angle(chain.rotation, actualRotation);
            Require(positionError < 0.000002f && rotationError < 0.05f, "Saved home chain disagrees with actual TCP.");
            var sceneLab = UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
            Require(sceneLab != null && sceneLab.RobotFrame == sceneLab.transform, "Scene prefab did not inherit RobotFrame.");
            CheckSaved(sceneLab);
            Require(hashes.All(p => Hash(p.Key) == p.Value), "An unrelated source asset changed.");
            report = new { passed = true, utc = DateTime.UtcNow.ToString("o"),
                scope = "Authored geometry, zero-pose composition and persisted configuration; not full FK/IK or real robot calibration",
                frame = "FanucRobot_Work root; Unity +X right, +Y up, +Z forward; model units interpreted as metres",
                transformOrder = "T(home position) * R(home) * R(local axis, q), followed by fixed A6->TCP",
                joints = lab.Joints.Select((j,i) => new { name = j.name, parent = j.transform.parent.name,
                    positionMetres = V(lab.Geometry.Joints[i].PositionMetres),
                    homeRotationXyzw = Q(lab.Geometry.Joints[i].Rotation),
                    localRotationAxis = V(j.LocalRotationAxis), minimumDegrees = j.MinimumDegrees,
                    maximumDegrees = j.MaximumDegrees, speedDegreesPerSecond = j.SpeedDegreesPerSecond }).ToArray(),
                toolPositionMetres = V(lab.Geometry.ToolPositionMetres), toolRotationXyzw = Q(lab.Geometry.ToolRotation),
                homeTcpPositionMetres = V(actualPosition), homeTcpRotationXyzw = Q(actualRotation),
                homePositionErrorMetres = positionError, homeRotationErrorDegrees = rotationError,
                persistedAndReloaded = true, modelAndControlSettingsPreserved = true,
                unrelatedSourceHashes = hashes, unrelatedSourcesUnchanged = true,
                limitsAreSimulationOnly = true, scaleIsUnit = true, realRobotModelAndDimensionsVerified = false };
            Directory.CreateDirectory("Logs/LW07A");
            File.WriteAllText("Logs/LW07A/geometry-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
        return report;
    }

    private static void CheckModel(FanucJointLab lab)
    {
        Require(lab != null && lab.Joints.Count == 6 && lab.Tcp != null, "Assign six joints and TCP.");
        Transform parent = lab.transform;
        Require(Vector3.Distance(parent.localScale, Vector3.one) < 0.000001f, "Root scale must be one.");
        for (int i = 0; i < 6; i++)
        {
            var joint = lab.Joints[i];
            Require(joint != null && joint.name == "A" + (i + 1) && joint.transform.parent == parent,
                "Expected direct root/A1/A2/A3/A4/A5/A6 chain.");
            Require(Vector3.Distance(joint.LocalRotationAxis, Axes[i]) < 0.000001f, "Unexpected rotary axis/sign.");
            Require(joint.MinimumDegrees == -45f && joint.MaximumDegrees == 45f && joint.SpeedDegreesPerSecond == 20f,
                "Existing simulation limits/speeds changed.");
            Require(Vector3.Distance(joint.transform.localScale, Vector3.one) < 0.000001f, "Joint scale must be one.");
            parent = joint.transform;
        }
        Require(lab.Tcp.parent == parent && Vector3.Distance(lab.Tcp.localScale, Vector3.one) < 0.000001f,
            "Expected unit-scaled TCP directly under A6.");
    }

    private static void CheckSaved(FanucJointLab lab)
    {
        CheckModel(lab);
        Require(lab.RobotFrame == lab.transform && lab.Geometry.Joints.Count == 6, "Missing saved geometry/frame.");
        for (int i = 0; i < 6; i++)
        {
            var saved = lab.Geometry.Joints[i];
            var source = lab.Joints[i].transform;
            Require(saved != null && Vector3.Distance(saved.PositionMetres, source.localPosition) < 0.000001f &&
                Quaternion.Angle(saved.Rotation, source.localRotation) < 0.05f &&
                Vector3.Distance(saved.LocalRotationAxis, lab.Joints[i].LocalRotationAxis) < 0.000001f,
                "Saved geometry disagrees with authored home: " + source.name);
        }
        Require(Vector3.Distance(lab.Geometry.ToolPositionMetres, lab.Tcp.localPosition) < 0.000001f &&
            Quaternion.Angle(lab.Geometry.ToolRotation, lab.Tcp.localRotation) < 0.05f, "Saved tool transform changed.");
    }

    private static string ModelFingerprint(FanucJointLab lab) => JsonConvert.SerializeObject(new {
        transforms = lab.GetComponentsInChildren<Transform>(true).Select(t => new { t.name,
            parent = t.parent == null ? null : t.parent.name, p = V(t.localPosition), r = Q(t.localRotation), s = V(t.localScale) }),
        axes = lab.Joints.Select(j => new { axis = V(j.LocalRotationAxis), j.MinimumDegrees, j.MaximumDegrees, j.SpeedDegreesPerSecond }) });
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion r) => new[] { r.x, r.y, r.z, r.w };
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
