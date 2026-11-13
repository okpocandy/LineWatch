using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class VerifyMachineControl
{
    public static async Task<object> Run()
    {
        if (!EditorApplication.isPlaying || SceneManager.GetActiveScene().name != "RobotLab")
            throw new InvalidOperationException("Open RobotLab and enter Play Mode before verification.");
        foreach (var root in SceneManager.GetActiveScene().GetRootGameObjects())
            foreach (var behaviour in root.GetComponentsInChildren<MonoBehaviour>(true))
                Require(behaviour != null && !(behaviour.GetType().Namespace ?? "").StartsWith("realvirtual"),
                    "RobotLab must not contain vendor behaviours or missing scripts.");
        var lab = UnityEngine.Object.FindFirstObjectByType<MachineControlLab>();
        Require(lab != null && lab.IsReady, "Connection lab must be ready.");
        var axis = lab.Axis;
        var token = lab.destroyCancellationToken;
        var results = new List<object>();
        int firstFrame = Time.frameCount;
        float firstGameTime = Time.time;
        try
        {
            lab.MoveProbeOutside();
            lab.MoveHome();
            await Until(() => AtTarget(lab, 0f) && !lab.TestSensor.Occupied, token);
            Quaternion homeRotation = axis.transform.localRotation;
            int firstEnter = lab.TestSensor.EnterCount;
            int firstExit = lab.TestSensor.ExitCount;

            for (int cycle = 1; cycle <= 3; cycle++)
            {
                int enterBefore = lab.TestSensor.EnterCount;
                int exitBefore = lab.TestSensor.ExitCount;
                float motionStart = Time.fixedTime;
                lab.MoveToTestAngle();
                await Until(() => axis.CurrentPosition > 3f, token);
                Require(axis.IsMoving && axis.CurrentPosition < lab.TestAngleDegrees - 1f,
                    "Axis must visibly move through intermediate poses.");
                float speedLimitError = axis.CurrentPosition - axis.SpeedDegreesPerSecond * (Time.fixedTime - motionStart);
                Require(speedLimitError < 0.05f, "Axis exceeded its configured speed limit.");
                await Until(() => AtTarget(lab, lab.TestAngleDegrees), token);
                await Settle(token);
                float targetError = Mathf.Abs(axis.CurrentPosition - lab.TestAngleDegrees);
                float transformError = Quaternion.Angle(homeRotation * Quaternion.AngleAxis(lab.TestAngleDegrees, Vector3.up),
                    axis.transform.localRotation);
                Require(transformError < 0.1f, "Axis value and actual joint rotation disagree.");

                lab.MoveHome();
                await Until(() => axis.CurrentPosition < lab.TestAngleDegrees - 3f, token);
                Require(!axis.IsAtTarget && axis.CurrentPosition > 1f, "Stop must be tested during movement.");
                lab.StopAxis();
                float stoppedAt = axis.CurrentPosition;
                Quaternion stoppedRotation = axis.transform.localRotation;
                await Settle(token);
                float stopDrift = Mathf.Abs(axis.CurrentPosition - stoppedAt);
                float stopTransformDrift = Quaternion.Angle(stoppedRotation, axis.transform.localRotation);
                Require(!axis.IsMoving && !axis.IsAtTarget && stopDrift < 0.01f && stopTransformDrift < 0.1f,
                    "Axis did not remain stopped short of its target.");
                lab.MoveHome();
                await Until(() => AtTarget(lab, 0f), token);
                await Settle(token);
                float homeError = Quaternion.Angle(homeRotation, axis.transform.localRotation);
                Require(homeError < 0.1f, "Joint did not return to its actual home rotation.");

                lab.MoveProbeInside();
                await Until(() => lab.TestSensor.Occupied && lab.TestSensor.EnterCount == enterBefore + 1, token);
                Require(lab.TestSensor.DetectedObject == lab.Probe.gameObject, "Sensor detected an unexpected object.");
                await Settle(token);
                Require(lab.TestSensor.EnterCount == enterBefore + 1 && lab.TestSensor.ExitCount == exitBefore,
                    "Stationary probe produced duplicate sensor transitions.");
                lab.MoveProbeOutside();
                await Until(() => !lab.TestSensor.Occupied && lab.TestSensor.ExitCount == exitBefore + 1, token);
                await Settle(token);
                Require(lab.TestSensor.EnterCount == enterBefore + 1 && lab.TestSensor.ExitCount == exitBefore + 1,
                    "Sensor transition counts do not match one entry and exit.");
                results.Add(new { cycle, targetErrorDegrees = targetError, transformErrorDegrees = transformError,
                    stoppedAtDegrees = stoppedAt, stopDriftDegrees = stopDrift, stopTransformDriftDegrees = stopTransformDrift,
                    homeErrorDegrees = homeError, speedLimitErrorDegrees = speedLimitError,
                    enterCount = lab.TestSensor.EnterCount, exitCount = lab.TestSensor.ExitCount });
            }

            int entries = lab.TestSensor.EnterCount - firstEnter;
            int exits = lab.TestSensor.ExitCount - firstExit;
            Require(!axis.MoveTo(float.NaN) && !axis.MoveTo(float.PositiveInfinity) && !axis.MoveTo(171f),
                "Non-finite and out-of-limit targets must be rejected.");
            Require(AtTarget(lab, 0f), "Invalid targets changed axis state.");
            Require(axis.MoveTo(-15f), "Negative target was rejected.");
            await Until(() => AtTarget(lab, -15f), token);
            Require(Quaternion.Angle(homeRotation * Quaternion.AngleAxis(-15f, Vector3.up), axis.transform.localRotation) < 0.1f,
                "Signed joint rotation is incorrect.");
            lab.MoveHome();
            await Until(() => AtTarget(lab, 0f), token);
            await VerifySensorLifecycle(lab, token);

            Require(Time.frameCount > firstFrame && Time.time > firstGameTime, "Player loop did not advance.");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"), scene = "RobotLab", cycles = results,
                implementation = "LineWatch own code; Unity Transform and PhysX", entries, exits,
                invalidTargetsRejected = true, signedRotationPassed = true, sensorLifecyclePassed = true,
                firstFrame, lastFrame = Time.frameCount, elapsedGameSeconds = Time.time - firstGameTime };
            string directory = Path.GetFullPath(Path.Combine(Application.dataPath, "../Logs/LW03-OwnCode"));
            Directory.CreateDirectory(directory);
            string reportPath = Path.Combine(directory, "verification-" + DateTime.UtcNow.ToString("yyyyMMdd-HHmmss") + ".json");
            File.WriteAllText(reportPath, JsonConvert.SerializeObject(report, Formatting.Indented));
            UnityEngine.Debug.Log("[LW-03] Three own-code axis/stop/sensor cycles and lifecycle checks passed. " + reportPath);
            return report;
        }
        finally
        {
            if (lab != null && Application.isPlaying && lab.IsReady)
            {
                lab.StopAxis();
                lab.MoveProbeOutside();
            }
        }
    }

    private static async Task VerifySensorLifecycle(MachineControlLab lab, CancellationToken token)
    {
        var sensor = lab.TestSensor;
        var primary = lab.Probe.GetComponent<BoxCollider>();
        BoxCollider extra = null;
        try
        {
            extra = lab.Probe.gameObject.AddComponent<BoxCollider>();
            extra.size = primary.size;
            int entries = sensor.EnterCount;
            int exits = sensor.ExitCount;
            lab.MoveProbeInside();
            await Until(() => sensor.Occupied, token);
            await Settle(token);
            Require(sensor.EnterCount == entries + 1, "Compound object produced duplicate entries.");
            primary.enabled = false;
            await Settle(token);
            Require(sensor.Occupied && sensor.ExitCount == exits, "One remaining collider must retain occupancy.");
            extra.enabled = false;
            await Until(() => !sensor.Occupied, token);
            Require(sensor.ExitCount == exits + 1 && sensor.DetectedObject == null, "Disabled colliders left stale occupancy.");
            extra.enabled = true;
            await Until(() => sensor.Occupied, token);
            UnityEngine.Object.Destroy(extra);
            extra = null;
            await Until(() => !sensor.Occupied, token);
            Require(sensor.DetectedObject == null, "Destroyed collider left stale occupancy.");
            primary.enabled = true;
            await Until(() => sensor.Occupied, token);
            sensor.enabled = false;
            Require(!sensor.Occupied, "Disabled sensor retained occupancy.");
            sensor.enabled = true;
            await Until(() => sensor.Occupied, token);
            lab.Probe.gameObject.SetActive(false);
            await Until(() => !sensor.Occupied, token);
            lab.Probe.gameObject.SetActive(true);
            await Until(() => sensor.Occupied, token);
            lab.MoveProbeOutside();
            await Until(() => !sensor.Occupied, token);
        }
        finally
        {
            sensor.enabled = true;
            primary.enabled = true;
            lab.Probe.gameObject.SetActive(true);
            if (extra != null) UnityEngine.Object.Destroy(extra);
        }
    }

    private static bool AtTarget(MachineControlLab lab, float target)
    {
        return lab.Axis.IsAtTarget && !lab.Axis.IsMoving && Mathf.Abs(lab.Axis.CurrentPosition - target) < 0.05f;
    }

    private static async Task Until(Func<bool> condition, CancellationToken token)
    {
        var watch = Stopwatch.StartNew();
        while (!condition())
        {
            token.ThrowIfCancellationRequested();
            if (watch.Elapsed.TotalSeconds > 8)
                throw new TimeoutException("LW-03 condition was not met within eight seconds.");
            await Task.Delay(20, token);
        }
    }

    private static async Task Settle(CancellationToken token)
    {
        float fixedStart = Time.fixedTime;
        int frameStart = Time.frameCount;
        await Until(() => Time.fixedTime - fixedStart >= Time.fixedDeltaTime * 3f && Time.frameCount > frameStart, token);
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
