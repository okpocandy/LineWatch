using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class VerifyFanucJoints
{
    public static async Task<object> Run()
    {
        if (!EditorApplication.isPlaying || EditorApplication.isPaused)
            throw new InvalidOperationException("Run RobotLab in unpaused Play Mode.");
        var lab = UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        Require(lab != null && lab.IsReady, "FANUC joint lab is not ready.");
        var token = lab.destroyCancellationToken;
        var directions = new[] { Vector3.down, Vector3.back, Vector3.back, Vector3.left, Vector3.back, Vector3.left };
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/03.Prefabs/FanucRobot_Work.prefab");
        var prefabJoint = prefab.transform;
        var homes = new Quaternion[6];
        for (int i = 0; i < 6; i++)
        {
            prefabJoint = prefabJoint.Find("A" + (i + 1));
            homes[i] = prefabJoint.localRotation;
            Require(Vector3.Distance(lab.Joints[i].LocalRotationAxis, directions[i]) < 0.0001f,
                "Configured joint direction disagrees with the joint table.");
        }
        var results = new List<object>();
        try
        {
            lab.HomeAll();
            await Until(() => lab.Joints.All(j => At(j, 0f)), token);
            for (int i = 0; i < 6; i++)
            {
                var axis = lab.Joints[i];
                Quaternion tcpHome = lab.Tcp.rotation;
                float startFixed = Time.fixedTime;
                Require(lab.MoveJoint(i, 15f), "Positive target was rejected.");
                await Until(() => axis.CurrentPosition >= 3f, token);
                Require(axis.IsMoving && axis.CurrentPosition < 14f, "Joint skipped intermediate poses.");
                float speedExcess = axis.CurrentPosition - 20f * (Time.fixedTime - startFixed);
                Require(speedExcess < 0.05f, "Joint exceeded the configured speed.");
                await Until(() => At(axis, 15f), token);
                float positiveError = Quaternion.Angle(homes[i] * Quaternion.AngleAxis(15f, directions[i]), axis.transform.localRotation);
                Require(positiveError < 0.05f, "Positive rotation has the wrong axis or sign.");
                float tcpRotationDelta = Quaternion.Angle(tcpHome, lab.Tcp.rotation);
                Require(Mathf.Abs(tcpRotationDelta - 15f) < 0.05f, "Actual TCP did not follow the joint rotation.");
                for (int j = 0; j < 6; j++)
                    if (j != i) Require(Quaternion.Angle(homes[j], lab.Joints[j].transform.localRotation) < 0.05f,
                        "Single-axis command changed another joint's local pose.");

                lab.MoveJoint(i, 0f);
                await Until(() => axis.CurrentPosition < 12f, token);
                Require(axis.IsMoving && axis.CurrentPosition > 1f, "Stop was not tested during movement.");
                axis.Stop();
                float stoppedAt = axis.CurrentPosition;
                Quaternion stoppedRotation = axis.transform.localRotation;
                await Settle(token);
                float stopDrift = Quaternion.Angle(stoppedRotation, axis.transform.localRotation);
                Require(!axis.IsMoving && !axis.IsAtTarget && Mathf.Abs(axis.CurrentPosition - stoppedAt) < 0.001f && stopDrift < 0.05f,
                    "Stopped joint did not hold its actual pose.");
                lab.MoveJoint(i, 0f);
                await Until(() => At(axis, 0f), token);
                Require(lab.MoveJoint(i, -15f), "Negative target was rejected.");
                await Until(() => At(axis, -15f), token);
                float negativeError = Quaternion.Angle(homes[i] * Quaternion.AngleAxis(-15f, directions[i]), axis.transform.localRotation);
                Require(negativeError < 0.05f, "Negative rotation has the wrong axis or sign.");
                Require(!lab.MoveJoint(i, 46f) && !lab.MoveJoint(i, float.NaN) && !lab.MoveJoint(i, float.PositiveInfinity),
                    "Out-of-range or non-finite targets were accepted.");
                Require(At(axis, -15f), "Rejected targets changed joint state.");
                lab.MoveJoint(i, 0f);
                await Until(() => At(axis, 0f), token);
                float homeError = Quaternion.Angle(homes[i], axis.transform.localRotation);
                Require(homeError < 0.05f, "Joint failed to restore its authored home rotation.");
                results.Add(new { joint = axis.name, positiveErrorDegrees = positiveError, negativeErrorDegrees = negativeError,
                    stoppedAtDegrees = stoppedAt, stopDriftDegrees = stopDrift, homeErrorDegrees = homeError,
                    tcpRotationDeltaDegrees = tcpRotationDelta, speedExcessDegrees = speedExcess });
            }
            lab.MoveJoint(0, 10f);
            lab.MoveJoint(5, -10f);
            await Until(() => lab.Joints[0].CurrentPosition > 3f && lab.Joints[5].CurrentPosition < -3f, token);
            lab.StopAll();
            var held = lab.Joints.Select(j => j.transform.localRotation).ToArray();
            await Settle(token);
            Require(lab.Joints.All(j => !j.IsMoving), "Stop All left a joint moving.");
            for (int i = 0; i < 6; i++)
                Require(Quaternion.Angle(held[i], lab.Joints[i].transform.localRotation) < 0.05f, "Stop All did not hold the model pose.");
            lab.HomeAll();
            await Until(() => lab.Joints.All(j => At(j, 0f)), token);
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"), joints = results,
                positiveAndNegativeTargets = 12, individualStops = 6, stopAllPassed = true, homeAllPassed = true,
                invalidTargetsRejected = true, scope = "Manual joint actuation only; no FK, IK or collision safety validation" };
            Directory.CreateDirectory("Logs/FanucJoints");
            File.WriteAllText("Logs/FanucJoints/verification-" + DateTime.UtcNow.ToString("yyyyMMdd-HHmmss") + ".json",
                JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;
        }
        finally { if (lab != null && Application.isPlaying) lab.StopAll(); }
    }

    private static bool At(RotaryAxis axis, float target) => !axis.IsMoving && axis.IsAtTarget && Mathf.Abs(axis.CurrentPosition - target) < 0.001f;
    private static async Task Settle(CancellationToken token)
    {
        float start = Time.fixedTime;
        await Until(() => Time.fixedTime - start >= Time.fixedDeltaTime * 4f, token);
    }
    private static async Task Until(Func<bool> condition, CancellationToken token)
    {
        var watch = Stopwatch.StartNew();
        while (!condition())
        {
            token.ThrowIfCancellationRequested();
            if (EditorApplication.isPaused) throw new InvalidOperationException("Verification interrupted by Editor pause.");
            if (watch.Elapsed.TotalSeconds > 6) throw new TimeoutException("FANUC did not reach the expected state in six seconds.");
            await Task.Delay(20, token);
        }
    }
    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
