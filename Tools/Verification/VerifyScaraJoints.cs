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

public static class VerifyScaraJoints
{
    public static async Task<object> Run()
    {
        if (!EditorApplication.isPlaying || EditorApplication.isPaused)
            throw new InvalidOperationException("Run RobotLab in unpaused Play Mode.");
        var lab = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        Require(lab != null && lab.IsReady, "SCARA four-axis lab is not ready.");
        var token = lab.destroyCancellationToken;
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/03.Prefabs/ScaraRobot_Work.prefab");
        var home1 = prefab.transform.Find("Base/Axis1");
        var home2 = home1.Find("Axis2");
        var home3 = home2.Find("Axis3");
        var home4 = home3.Find("Axis4");
        var authored = new[] { home1, home2, home3, home4 };
        var homeRotations = authored.Select(t => t.localRotation).ToArray();
        var homePositions = authored.Select(t => t.localPosition).ToArray();
        var axes = new[] { lab.Axis1.transform, lab.Axis2.transform, lab.Axis3.transform, lab.Axis4.transform };
        var rotary = new[] { lab.Axis1, lab.Axis2, null, lab.Axis4 };
        var limits = new[] { 170f, 150f, 0f, 180f };
        var results = new List<object>();
        try
        {
            lab.HomeAll();
            await Until(() => AllHome(lab), token);
            Require(Vector3.Distance(lab.ActualTcpPosition, new Vector3(0.7f, -0.115177f, 0f)) < 0.000002f,
                "Actual home TCP does not match the authored reference layout.");
            foreach (int i in new[] { 0, 1, 3 })
            {
                var axis = rotary[i];
                Require(Vector3.Distance(axis.LocalRotationAxis, Vector3.up) < 0.0001f &&
                    axis.MinimumDegrees == -limits[i] && axis.MaximumDegrees == limits[i], "Rotary joint table does not match configuration.");
                Require(lab.MoveJoint(i, -limits[i]), "Lower boundary command was rejected.");
                axis.Stop();
                Require(lab.MoveJoint(i, limits[i]), "Upper boundary command was rejected.");
                axis.Stop();
                Require(!lab.MoveJoint(i, limits[i] + 1f) && !lab.MoveJoint(i, -limits[i] - 1f) &&
                    !lab.MoveJoint(i, float.NaN) && !lab.MoveJoint(i, float.PositiveInfinity), "Invalid rotary targets were accepted.");
                lab.MoveJoint(i, 0f);
                Vector3 tcpHome = lab.Tcp.position;
                Quaternion tcpHomeRotation = lab.Tcp.rotation;
                Vector3 pivot = axis.transform.position;
                Vector3 worldAxis = axis.transform.TransformDirection(Vector3.up);
                float startFixed = Time.fixedTime;
                lab.MoveJoint(i, 20f);
                await Until(() => axis.CurrentPosition >= 3f, token);
                Require(axis.IsMoving && axis.CurrentPosition < 19f, "Rotary joint skipped intermediate poses.");
                float speedExcess = axis.CurrentPosition - 30f * (Time.fixedTime - startFixed);
                Require(speedExcess < 0.05f, "Rotary joint exceeded its configured speed.");
                await Until(() => At(axis, 20f), token);
                float positiveError = Quaternion.Angle(homeRotations[i] * Quaternion.AngleAxis(20f, Vector3.up), axes[i].localRotation);
                float tcpPositionError = Vector3.Distance(pivot + Quaternion.AngleAxis(20f, worldAxis) * (tcpHome - pivot), lab.Tcp.position);
                float tcpRotationError = Mathf.Abs(Quaternion.Angle(tcpHomeRotation, lab.Tcp.rotation) - 20f);
                Require(positiveError < 0.05f && tcpPositionError < 0.000002f && tcpRotationError < 0.05f,
                    "Actual rotary joint or TCP did not follow the commanded axis and sign.");
                Require(Mathf.Abs(lab.ActualTcpYawDegrees - 20f) < 0.05f, "TCP yaw readout has the wrong sign.");
                CheckOtherLocalPoses(axes, homePositions, homeRotations, i);
                lab.MoveJoint(i, 0f);
                await Until(() => axis.CurrentPosition < 16f, token);
                Require(axis.IsMoving && axis.CurrentPosition > 1f, "Rotary stop was not tested during movement.");
                lab.StopJoint(i);
                float stoppedAt = axis.CurrentPosition;
                Quaternion stoppedRotation = axes[i].localRotation;
                await Settle(token);
                float stopDrift = Quaternion.Angle(stoppedRotation, axes[i].localRotation);
                Require(!axis.IsMoving && !axis.IsAtTarget && Mathf.Abs(stoppedAt - axis.CurrentPosition) < 0.001f && stopDrift < 0.05f,
                    "Rotary stop did not hold the model pose.");
                lab.MoveJoint(i, 0f);
                await Until(() => At(axis, 0f), token);
                lab.MoveJoint(i, -20f);
                await Until(() => At(axis, -20f), token);
                float negativeError = Quaternion.Angle(homeRotations[i] * Quaternion.AngleAxis(-20f, Vector3.up), axes[i].localRotation);
                Require(negativeError < 0.05f && Mathf.Abs(lab.ActualTcpYawDegrees + 20f) < 0.05f, "Negative rotary motion is incorrect.");
                lab.MoveJoint(i, 0f);
                await Until(() => At(axis, 0f), token);
                float homeError = Quaternion.Angle(homeRotations[i], axes[i].localRotation);
                Require(homeError < 0.05f, "Rotary joint did not return to its authored home.");
                results.Add(new { axis = axis.name, positiveErrorDegrees = positiveError, negativeErrorDegrees = negativeError,
                    tcpPositionErrorMetres = tcpPositionError, tcpRotationErrorDegrees = tcpRotationError,
                    stopDriftDegrees = stopDrift, homeErrorDegrees = homeError, speedExcessDegrees = speedExcess });
            }

            var slide = lab.Axis3;
            Require(slide.MinimumMetres == 0f && Mathf.Abs(slide.MaximumMetres - 0.15f) < 0.000001f,
                "Linear range disagrees with the joint table.");
            Vector3 tcpStart = lab.Tcp.position;
            Quaternion tcpStartRotation = lab.Tcp.rotation;
            float linearStart = Time.fixedTime;
            lab.MoveJoint(2, 0.05f);
            await Until(() => slide.CurrentPosition > 0.01f, token);
            Require(slide.IsMoving && slide.CurrentPosition < 0.049f, "Slide skipped intermediate poses.");
            float linearSpeedExcess = slide.CurrentPosition - 0.05f * (Time.fixedTime - linearStart);
            Require(linearSpeedExcess < 0.00001f, "Slide exceeded its configured speed.");
            await Until(() => At(slide, 0.05f), token);
            float slidePositionError = Vector3.Distance(axes[2].localPosition, homePositions[2] + Vector3.down * 0.05f);
            float slideTcpError = Vector3.Distance(lab.Tcp.position, tcpStart - lab.RobotFrame.up * 0.05f);
            Require(slidePositionError < 0.000002f && slideTcpError < 0.000002f &&
                Quaternion.Angle(tcpStartRotation, lab.Tcp.rotation) < 0.05f, "50mm downward motion changed the wrong coordinate or rotation.");
            CheckOtherLocalPoses(axes, homePositions, homeRotations, 2);
            lab.MoveJoint(2, 0f);
            await Until(() => slide.CurrentPosition < 0.04f, token);
            Require(slide.IsMoving && slide.CurrentPosition > 0.001f, "Slide stop was not tested during movement.");
            lab.StopJoint(2);
            float stoppedSlideAt = slide.CurrentPosition;
            Vector3 stoppedSlidePosition = axes[2].localPosition;
            await Settle(token);
            float slideStopDrift = Vector3.Distance(stoppedSlidePosition, axes[2].localPosition);
            Require(!slide.IsMoving && !slide.IsAtTarget && Mathf.Abs(slide.CurrentPosition - stoppedSlideAt) < 0.000001f &&
                slideStopDrift < 0.000001f, "Slide did not hold after stop.");
            lab.MoveJoint(2, 0f);
            await Until(() => At(slide, 0f), token);
            Require(lab.MoveJoint(2, 0.15f), "150mm stroke endpoint was rejected.");
            await Until(() => At(slide, 0.15f), token);
            float fullStrokeError = Vector3.Distance(homePositions[2] + Vector3.down * 0.15f, axes[2].localPosition);
            Require(fullStrokeError < 0.000002f, "Slide did not reach its 150mm endpoint.");
            Require(!lab.MoveJoint(2, -0.001f) && !lab.MoveJoint(2, 0.151f) && !lab.MoveJoint(2, float.NaN) &&
                !lab.MoveJoint(2, float.PositiveInfinity) && At(slide, 0.15f), "Invalid slide targets changed the axis state.");
            lab.HomeAll();
            await Until(() => AllHome(lab), token);
            lab.MoveJoint(0, 10f); lab.MoveJoint(1, -10f); lab.MoveJoint(2, 0.03f); lab.MoveJoint(3, 10f);
            await Until(() => lab.Axis1.CurrentPosition > 2f && lab.Axis2.CurrentPosition < -2f &&
                slide.CurrentPosition > 0.003f && lab.Axis4.CurrentPosition > 2f, token);
            lab.StopAll();
            var heldPositions = axes.Select(t => t.localPosition).ToArray();
            var heldRotations = axes.Select(t => t.localRotation).ToArray();
            await Settle(token);
            Require(!lab.Axis1.IsMoving && !lab.Axis2.IsMoving && !slide.IsMoving && !lab.Axis4.IsMoving, "Stop All left an axis moving.");
            for (int i = 0; i < axes.Length; i++)
                Require(Vector3.Distance(heldPositions[i], axes[i].localPosition) < 0.000001f &&
                    Quaternion.Angle(heldRotations[i], axes[i].localRotation) < 0.05f, "Stop All changed an actual joint pose.");
            lab.HomeAll();
            await Until(() => AllHome(lab), token);
            float finalHomeTcpError = Vector3.Distance(lab.ActualTcpPosition, new Vector3(0.7f, -0.115177f, 0f));
            Require(finalHomeTcpError < 0.000002f && Mathf.Abs(lab.ActualTcpYawDegrees) < 0.05f, "Home All failed to restore the actual TCP.");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"), rotaryAxes = results,
                slide = new { targetMetres = 0.05f, positionErrorMetres = slidePositionError, tcpErrorMetres = slideTcpError,
                    stopDriftMetres = slideStopDrift, fullStrokeErrorMetres = fullStrokeError, speedExcessMetres = linearSpeedExcess },
                stopAllPassed = true, homeAllPassed = true, invalidTargetsRejected = true, finalHomeTcpErrorMetres = finalHomeTcpError,
                scope = "LW-04c individual axis actuation and actual Transform/TCP; no FK solver, IK or collision safety verification" };
            Directory.CreateDirectory("Logs/ScaraJoints");
            File.WriteAllText("Logs/ScaraJoints/verification-" + DateTime.UtcNow.ToString("yyyyMMdd-HHmmss") + ".json",
                JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;
        }
        finally { if (lab != null && Application.isPlaying) lab.StopAll(); }
    }

    private static bool At(RotaryAxis axis, float target) => !axis.IsMoving && axis.IsAtTarget && Mathf.Abs(axis.CurrentPosition - target) < 0.001f;
    private static bool At(LinearAxis axis, float target) => !axis.IsMoving && axis.IsAtTarget && Mathf.Abs(axis.CurrentPosition - target) < 0.000001f;
    private static bool AllHome(ScaraJointLab lab) => At(lab.Axis1, 0f) && At(lab.Axis2, 0f) && At(lab.Axis3, 0f) && At(lab.Axis4, 0f);
    private static void CheckOtherLocalPoses(Transform[] axes, Vector3[] positions, Quaternion[] rotations, int except)
    {
        for (int i = 0; i < axes.Length; i++)
            if (i != except) Require(Vector3.Distance(positions[i], axes[i].localPosition) < 0.000002f &&
                Quaternion.Angle(rotations[i], axes[i].localRotation) < 0.05f, "Single-axis command changed another joint's local pose.");
    }
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
            if (watch.Elapsed.TotalSeconds > 7) throw new TimeoutException("SCARA did not reach the expected state in seven seconds.");
            await Task.Delay(20, token);
        }
    }
    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
