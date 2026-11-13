using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class VerifyScaraForwardKinematicsFull
{
    public static object Run()
    {
        const string prefabPath = "Assets/03.Prefabs/ScaraRobot_Work.prefab";
        const string scenePath = "Assets/01.Scenes/RobotLab.unity";
        const float positionToleranceMetres = 0.00001f;
        const float angleToleranceDegrees = 0.005f;
        const int seed = 20261114;
        string prefabHash = Hash(prefabPath), sceneHash = Hash(scenePath);
        var root = PrefabUtility.LoadPrefabContents(prefabPath);
        try
        {
            var frame = root.transform.Find("RobotFrame");
            var axis1 = root.transform.Find("Base/Axis1");
            var axis2 = axis1.Find("Axis2");
            var axis3 = axis2.Find("Axis3");
            var axis4 = axis3.Find("Axis4");
            var gripper = axis4.Find("Gripper");
            var tcp = gripper.Find("TCP");
            var home1 = axis1.localRotation;
            var home2 = axis2.localRotation;
            var home3 = axis3.localPosition;
            var home4 = axis4.localRotation;
            var homeGripperPosition = gripper.localPosition;
            var homeGripperRotation = gripper.localRotation;
            var rootPosition = root.transform.position;
            var rootRotation = root.transform.rotation;
            var placements = new[] {
                new { name = "Authored", position = rootPosition, rotation = rootRotation },
                new { name = "Translated and yawed", position = new Vector3(2.1f,1.2f,-3f), rotation = Quaternion.Euler(0f,37f,0f) },
                new { name = "Translated and tilted", position = new Vector3(-4.2f,0.8f,2.3f), rotation = Quaternion.Euler(20f,47f,-15f) }
            };
            // A3's authored +90-degree Y rotation maps gripper local (-z, y, x)
            // to RobotFrame (x, y, z) at zero joint values.
            var tools = new[] {
                new { name = "Original", gripperDelta = Vector3.zero, yaw = 0f, offset = new Vector3(0f,-0.034177f,0f) },
                new { name = "Mount 20mm lower", gripperDelta = new Vector3(0f,-0.02f,0f), yaw = 0f, offset = new Vector3(0f,-0.054177f,0f) },
                new { name = "Horizontal mount offset", gripperDelta = new Vector3(-0.01f,0f,0.02f), yaw = 0f, offset = new Vector3(0.02f,-0.034177f,0.01f) },
                new { name = "Combined mount and yaw", gripperDelta = new Vector3(-0.01f,-0.02f,0.02f), yaw = 35f, offset = new Vector3(0.02f,-0.054177f,0.01f) }
            };
            // x/y/z/w = q1 degrees, q2 degrees, d3 metres, q4 degrees.
            var poses = new List<Vector4> {
                new Vector4(0f,0f,0f,0f), new Vector4(90f,0f,0f,0f), new Vector4(-90f,0f,0f,0f),
                new Vector4(0f,90f,0f,0f), new Vector4(90f,-90f,0f,0f), new Vector4(0f,0f,0.05f,0f),
                new Vector4(0f,0f,0f,90f), new Vector4(45f,0f,0.05f,-45f),
                new Vector4(-170f,-150f,0f,-180f), new Vector4(170f,150f,0.15f,180f),
                new Vector4(0f,0f,0.15f,0f), new Vector4(0f,0.0001f,0.075f,180f),
                new Vector4(170f,-150f,0.15f,-180f), new Vector4(-170f,150f,0f,180f),
                new Vector4(0f,-150f,0.075f,-180f), new Vector4(0f,150f,0.075f,180f)
            };
            var random = new System.Random(seed);
            float Range(float min, float max) => min + (max - min) * (float)random.NextDouble();
            while (poses.Count < 100)
                poses.Add(new Vector4(Range(-170f,170f), Range(-150f,150f), Range(0f,0.15f), Range(-180f,180f)));

            void ApplyPose(Vector4 pose)
            {
                axis1.localRotation = home1 * Quaternion.AngleAxis(pose.x, Vector3.up);
                axis2.localRotation = home2 * Quaternion.AngleAxis(pose.y, Vector3.up);
                axis3.localPosition = home3 + Vector3.down * pose.z;
                axis4.localRotation = home4 * Quaternion.AngleAxis(pose.w, Vector3.up);
            }
            var groups = new List<object>();
            int failures = 0, comparisons = 0;
            var failureDetails = new List<object>();
            float maxLocalError = 0f, maxWorldError = 0f, maxYawError = 0f, maxRotationError = 0f, maxInvariantError = 0f;
            foreach (var tool in tools)
            {
                gripper.localPosition = homeGripperPosition + tool.gripperDelta;
                gripper.localRotation = homeGripperRotation * Quaternion.AngleAxis(tool.yaw, Vector3.up);
                var geometry = new ScaraForwardKinematics.Geometry(0.325f,0.375f,0.016783f,-0.097783f,tool.offset,tool.yaw);
                var baselinePositions = new Vector3[poses.Count];
                var baselineYaws = new float[poses.Count];
                for (int placementIndex = 0; placementIndex < placements.Length; placementIndex++)
                {
                    var placement = placements[placementIndex];
                    root.transform.SetPositionAndRotation(placement.position, placement.rotation);
                    float groupLocal = 0f, groupWorld = 0f, groupYaw = 0f, groupRotation = 0f, groupInvariant = 0f;
                    for (int i = 0; i < poses.Count; i++)
                    {
                        var pose = poses[i];
                        ApplyPose(pose);
                        var calculated = ScaraForwardKinematics.Calculate(pose.x,pose.y,pose.z,pose.w,geometry);
                        Vector3 actualLocal = frame.InverseTransformPoint(tcp.position);
                        float actualYaw = Vector3.SignedAngle(frame.forward,tcp.forward,frame.up);
                        float localError = Vector3.Distance(calculated.PositionMetres,actualLocal);
                        float worldError = Vector3.Distance(frame.TransformPoint(calculated.PositionMetres),tcp.position);
                        float yawError = Mathf.Abs(Mathf.DeltaAngle(calculated.YawDegrees,actualYaw));
                        var expectedRotation = frame.rotation * Quaternion.AngleAxis(calculated.YawDegrees,Vector3.up);
                        float rotationError = RotationError(expectedRotation,tcp.rotation);
                        float invariantError = 0f, invariantYaw = 0f;
                        if (placementIndex == 0) { baselinePositions[i] = actualLocal; baselineYaws[i] = actualYaw; }
                        else
                        {
                            invariantError = Vector3.Distance(baselinePositions[i],actualLocal);
                            invariantYaw = Mathf.Abs(Mathf.DeltaAngle(baselineYaws[i],actualYaw));
                        }
                        groupLocal = Mathf.Max(groupLocal,localError); groupWorld = Mathf.Max(groupWorld,worldError);
                        groupYaw = Mathf.Max(groupYaw,yawError); groupRotation = Mathf.Max(groupRotation,rotationError);
                        groupInvariant = Mathf.Max(groupInvariant,invariantError);
                        comparisons++;
                        if (localError > positionToleranceMetres || worldError > positionToleranceMetres ||
                            yawError > angleToleranceDegrees || rotationError > angleToleranceDegrees ||
                            invariantError > positionToleranceMetres || invariantYaw > angleToleranceDegrees)
                        {
                            failures++;
                            if (failureDetails.Count < 8) failureDetails.Add(new { tool = tool.name, placement = placement.name,
                                poseIndex = i, jointValues = Values(pose), localError, worldError, yawError, rotationError, invariantError, invariantYaw });
                        }
                    }
                    maxLocalError = Mathf.Max(maxLocalError,groupLocal); maxWorldError = Mathf.Max(maxWorldError,groupWorld);
                    maxYawError = Mathf.Max(maxYawError,groupYaw); maxRotationError = Mathf.Max(maxRotationError,groupRotation);
                    maxInvariantError = Mathf.Max(maxInvariantError,groupInvariant);
                    groups.Add(new { tool = tool.name, placement = placement.name, samples = poses.Count,
                        toolOffsetMetres = Values(tool.offset), homeToolYawDegrees = tool.yaw,
                        maxLocalPositionErrorMetres = groupLocal, maxWorldPositionErrorMetres = groupWorld,
                        maxYawErrorDegrees = groupYaw, maxWorldRotationErrorDegrees = groupRotation,
                        maxPlacementInvariantErrorMetres = groupInvariant });
                }
            }
            // A real mount change must be detectable when its FK input is not updated.
            root.transform.SetPositionAndRotation(rootPosition,rootRotation);
            gripper.localPosition = homeGripperPosition + Vector3.down * 0.02f;
            gripper.localRotation = homeGripperRotation;
            ApplyPose(Vector4.zero);
            var oldGeometry = new ScaraForwardKinematics.Geometry(0.325f,0.375f,0.016783f,-0.097783f,tools[0].offset);
            var oldResult = ScaraForwardKinematics.Calculate(0f,0f,0f,0f,oldGeometry);
            float uncalibratedToolError = Vector3.Distance(oldResult.PositionMetres,frame.InverseTransformPoint(tcp.position));
            bool mismatchDetected = Mathf.Abs(uncalibratedToolError - 0.02f) < positionToleranceMetres;
            bool prefabUnchanged = prefabHash == Hash(prefabPath), sceneUnchanged = sceneHash == Hash(scenePath);
            bool passed = failures == 0 && mismatchDetected && prefabUnchanged && sceneUnchanged;
            var report = new { passed, utc = DateTime.UtcNow.ToString("o"), seed,
                distinctJointPoses = poses.Count, placements = placements.Length, toolVariants = tools.Length,
                staticPoseComparisons = comparisons, failures, positionToleranceMetres, angleToleranceDegrees,
                maxLocalPositionErrorMetres = maxLocalError, maxWorldPositionErrorMetres = maxWorldError,
                maxLocalYawErrorDegrees = maxYawError, maxWorldRotationErrorDegrees = maxRotationError,
                maxPlacementInvariantErrorMetres = maxInvariantError, uncalibratedToolErrorMetres = uncalibratedToolError,
                mismatchDetected, sourcePrefabUnchanged = prefabUnchanged, sourceSceneUnchanged = sceneUnchanged,
                scope = "Isolated FK/Transform validation, not motor cycles, IK, collision or real robot accuracy",
                jointPoses = poses.Select(Values).ToArray(), groups, failureDetails };
            Directory.CreateDirectory("Logs/LW05");
            File.WriteAllText("Logs/LW05/full-verification.json",JsonConvert.SerializeObject(report,Formatting.Indented));
            if (!passed) throw new InvalidOperationException("Full FK verification failed; inspect Logs/LW05/full-verification.json.");
            return new { report.passed, report.distinctJointPoses, report.placements, report.toolVariants,
                report.staticPoseComparisons, report.failures, report.maxLocalPositionErrorMetres,
                report.maxWorldPositionErrorMetres, report.maxLocalYawErrorDegrees, report.maxWorldRotationErrorDegrees,
                report.maxPlacementInvariantErrorMetres, report.uncalibratedToolErrorMetres,
                report.sourcePrefabUnchanged, report.sourceSceneUnchanged };
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
    }

    private static float RotationError(Quaternion expected, Quaternion actual)
    {
        var delta = Quaternion.Inverse(expected) * actual;
        return 2f * Mathf.Atan2(new Vector3(delta.x,delta.y,delta.z).magnitude,Mathf.Abs(delta.w)) * Mathf.Rad2Deg;
    }
    private static float[] Values(Vector3 v) => new[] { v.x,v.y,v.z };
    private static float[] Values(Vector4 v) => new[] { v.x,v.y,v.z,v.w };
    private static string Hash(string path)
    {
        using (var sha = SHA256.Create()) return Convert.ToBase64String(sha.ComputeHash(File.ReadAllBytes(path)));
    }
}
