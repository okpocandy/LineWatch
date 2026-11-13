using System;
using System.IO;
using System.Linq;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class VerifyScaraFrames
{
    public static object Run()
    {
        var root = PrefabUtility.LoadPrefabContents("Assets/03.Prefabs/ScaraRobot_Work.prefab");
        try
        {
            var frame = root.transform.Find("RobotFrame");
            var axis1 = root.transform.Find("Base/Axis1");
            var axis2 = axis1.Find("Axis2");
            var axis3 = axis2.Find("Axis3");
            var axis4 = axis3.Find("Axis4");
            Require(frame != null && axis4 != null, "Robot frame and wrist must exist after prefab reload.");
            var gripper = axis4.Find("Gripper");
            var tcp = gripper != null ? gripper.Find("TCP") : null;
            Require(tcp != null && tcp.childCount == 0, "TCP must be a leaf below the gripper.");
            Require(frame.parent == root.transform, "Robot frame must stay independent of moving joints.");
            Require(Vector3.Distance(frame.position, axis1.position) < 0.000001f, "Frame origin must match the Axis1 home pivot.");
            Require(Vector3.Distance(axis4.localPosition, Vector3.zero) < 0.000001f &&
                Quaternion.Angle(axis4.localRotation, Quaternion.identity) < 0.001f, "Wrist home must preserve the authored gripper pose.");
            Require(Quaternion.Angle(tcp.rotation, frame.rotation) < 0.001f, "TCP home axes must align with the robot frame.");
            Require(root.GetComponentsInChildren<MeshRenderer>(true).Length == 6, "Visual mesh count changed.");
            Require(root.GetComponentsInChildren<Transform>(true)
                .Sum(t => GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(t.gameObject)) == 0, "Missing scripts found.");
            Require(root.GetComponentsInChildren<MonoBehaviour>(true)
                .All(c => c is LineWatch.RotaryAxis || c is LineWatch.LinearAxis || c is LineWatch.ScaraJointLab),
                "Unexpected motion components found.");
            var lab = root.GetComponent<LineWatch.ScaraJointLab>();
            Require(lab != null && lab.IsReady && lab.Axis1 == axis1.GetComponent<LineWatch.RotaryAxis>() &&
                lab.Axis2 == axis2.GetComponent<LineWatch.RotaryAxis>() && lab.Axis3 == axis3.GetComponent<LineWatch.LinearAxis>() &&
                lab.Axis4 == axis4.GetComponent<LineWatch.RotaryAxis>() && lab.RobotFrame == frame && lab.Tcp == tcp,
                "Persisted four-axis and frame connections are incomplete.");
            Require(root.GetComponentsInChildren<LineWatch.RotaryAxis>(true).Length == 3 &&
                root.GetComponentsInChildren<LineWatch.LinearAxis>(true).Length == 1, "Expected three rotary axes and one slide.");
            Vector3 expectedHome = new Vector3(0.7f, -0.115177f, 0f);
            Vector3 localHome = frame.InverseTransformPoint(tcp.position);
            float homeError = Vector3.Distance(localHome, expectedHome);
            Require(homeError < 0.000001f, "Persisted TCP home differs from the measured model geometry.");

            // Verify the reference-frame layout using only this disposable prefab copy.
            root.transform.SetPositionAndRotation(new Vector3(2.1f, 1.2f, -3f), Quaternion.Euler(0f, 37f, 0f));
            float relocatedError = Vector3.Distance(localHome, frame.InverseTransformPoint(tcp.position));
            Require(relocatedError < 0.000001f, "Moving the robot changed its reference-frame coordinates.");
            float relocatedRotationError = Quaternion.Angle(frame.rotation, tcp.rotation);
            Require(relocatedRotationError < 0.001f, "Moving the robot changed the TCP home orientation.");
            var report = new {
                passed = true, utc = DateTime.UtcNow.ToString("o"),
                stage = "LW-04c persisted layout and four-axis connections; no joint actuation in this check",
                tcpHomeInRobotFrame = new[] { localHome.x, localHome.y, localHome.z },
                homeErrorMetres = homeError, relocatedErrorMetres = relocatedError,
                relocatedRotationErrorDegrees = relocatedRotationError,
                savedPrefabReloadPassed = true, visualMeshCount = 6, missingScripts = 0
            };
            Directory.CreateDirectory("Logs/LW04");
            File.WriteAllText("Logs/LW04/frame-verification-with-controls.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
