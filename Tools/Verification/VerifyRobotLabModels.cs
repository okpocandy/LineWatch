using System;
using System.IO;
using System.Linq;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class VerifyRobotLabModels
{
    public static object Run()
    {
        var scene = SceneManager.GetActiveScene();
        Require(scene.path == "Assets/01.Scenes/RobotLab.unity", "Open RobotLab before verification.");
        var roots = scene.GetRootGameObjects();
        var fanuc = roots.Single(r => r.name == "FanucRobot_Work");
        var scara = roots.Single(r => r.name == "ScaraRobot_Work");
        var lab = UnityEngine.Object.FindFirstObjectByType<LineWatch.MachineControlLab>();
        Require(lab != null && lab.IsReady && lab.Axis.transform.IsChildOf(scara.transform),
            "The existing SCARA test connection changed.");
        var joint = fanuc.transform;
        for (int i = 1; i <= 6; i++)
        {
            joint = joint.Find("A" + i);
            Require(joint != null, "FANUC joint hierarchy is incomplete at A" + i);
        }
        Require(joint.Find("TCP/SchunkEGH80Gripper/LeftFinger") != null &&
            joint.Find("TCP/SchunkEGH80Gripper/RightFinger") != null, "Gripper visual hierarchy is incomplete.");
        var fanucLab = fanuc.GetComponent<LineWatch.FanucJointLab>();
        var fanucAxes = fanuc.GetComponentsInChildren<LineWatch.RotaryAxis>(true);
        Require(fanucLab != null && fanucLab.IsReady && fanucAxes.Length == 6 &&
            fanuc.GetComponentsInChildren<MonoBehaviour>(true).All(c => c is LineWatch.RotaryAxis || c is LineWatch.FanucJointLab) &&
            fanuc.GetComponentsInChildren<Rigidbody>(true).Length == 0 &&
            fanuc.GetComponentsInChildren<Collider>(true).Length == 0, "FANUC must use only its own manual joint controls.");
        var renderers = fanuc.GetComponentsInChildren<MeshRenderer>(true);
        Require(renderers.Length == 10 && renderers.All(r => r.GetComponent<MeshFilter>().sharedMesh != null &&
            r.sharedMaterials.All(m => m != null)), "FANUC meshes or materials are missing.");
        Require(scara.transform.Find("RobotFrame") != null &&
            scara.transform.Find("Base/Axis1/Axis2/Axis3/Axis4/Gripper/TCP") != null,
            "SCARA reference frames changed.");
        int missingScripts = roots.Sum(r => r.GetComponentsInChildren<Transform>(true)
            .Sum(t => GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(t.gameObject)));
        Require(missingScripts == 0, "Scene contains missing scripts.");
        var vendorCode = AssetDatabase.GetDependencies(scene.path, true).Where(p =>
            p.StartsWith("Packages/io.realvirtual.starter/") && (p.EndsWith(".cs") || p.EndsWith(".prefab"))).ToArray();
        Require(vendorCode.Length == 0, "Scene depends on vendor code or behaviour prefabs: " + string.Join(", ", vendorCode));
        var report = new {
            passed = true, utc = DateTime.UtcNow.ToString("o"), playMode = Application.isPlaying,
            robots = new[] { scara.name, fanuc.name }, fanucJoints = 6, fanucMeshes = renderers.Length,
            fanucOwnRotaryAxes = fanucAxes.Length, missingScripts, vendorCodeDependencies = vendorCode.Length,
            scaraLabReady = lab.IsReady, scaraFramesPreserved = true,
            scope = "Own manual joint controls connected; no FANUC FK or IK"
        };
        Directory.CreateDirectory("Logs/RobotModels");
        File.WriteAllText("Logs/RobotModels/verification-joints-" + (Application.isPlaying ? "play" : "editor") + ".json",
            JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
