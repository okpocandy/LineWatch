using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

public static class VerifyScaraIkControls
{
    private static readonly string[] SourcePaths = { "Assets/01.Scenes/RobotLab.unity", "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };

    public static object EditMode()
    {
        Require(!EditorApplication.isPlayingOrWillChangePlaymode, "Edit Mode required");
        var lab = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        Require(lab != null && lab.IsReady, "RobotLab references required");
        var before = Targets(lab);
        Require(!lab.TryMoveTcp(new Vector3(.4f,-.2f,-.3f), 0f, ScaraJointLab.IkSelection.Nearest, out _, out _), "Edit Mode command accepted");
        Require(Targets(lab) == before, "Edit Mode changed commands");
        Require(EditorApplication.ExecuteMenuItem("LineWatch/SCARA 4축 조작 테스트"), "Menu missing");
        var ui = EditorWindow.GetWindow<ScaraJointLabWindow>().rootVisualElement;
        Require(ui.Q<FloatField>("scara-ik-x").value == 400f && ui.Q<FloatField>("scara-ik-y").value == -200f &&
            ui.Q<FloatField>("scara-ik-z").value == -300f && ui.Q<FloatField>("scara-ik-yaw").value == 0f, "Default goal or units changed");
        Require(!ui.Q<Button>("scara-ik-move").enabledInHierarchy && !ui.Q<FloatField>("scara-ik-x").enabledInHierarchy, "Edit Mode controls enabled");
        Require(ui.Q<DropdownField>("scara-ik-selection").choices.Count == 3 && ui.Q<Button>("scara-ik-stop") != null &&
            ui.Q<Label>("scara-fk-calculated") != null, "IK or existing FK controls missing");
        var report = new { passed = true, editModeRejected = true, defaultInputsVerified = true, sourceHashes = SourcePaths.ToDictionary(path => path, Hash) };
        Directory.CreateDirectory("Logs/LW06");
        File.WriteAllText("Logs/LW06/controls-edit-mode.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;
    }

    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Unpaused RobotLab Play Mode required");
        var lab = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        Require(lab != null && lab.IsReady, "RobotLab references required");
        var token = lab.destroyCancellationToken;
        var settings = new SerializedObject(lab);
        float link1 = settings.FindProperty("fkLink1Metres").floatValue;
        var axisSettings = new SerializedObject(lab.Axis2);
        float maxAxis2 = axisSettings.FindProperty("maximumDegrees").floatValue;
        bool axis4Enabled = lab.Axis4.enabled;
        float originalTimeScale = Time.timeScale;
        var window = EditorWindow.GetWindow<ScaraJointLabWindow>();
        window.RefreshLab();
        var ui = window.rootVisualElement;
        var x = ui.Q<FloatField>("scara-ik-x");
        var selection = ui.Q<DropdownField>("scara-ik-selection");
        var move = ui.Q<Button>("scara-ik-move");
        var candidates = ui.Q<Label>("scara-ik-candidates");
        var chosen = ui.Q<Label>("scara-ik-selected");
        var feedback = ui.Q<HelpBox>("scara-ik-feedback");
        var goal = new Vector3(.4f,-.2f,-.3f);
        var home = new Vector3(.7f,-.115177f,0f);
        int rejected = 0, actualArrivals = 0, movingSamples = 0;
        float maxPositionError = 0f, maxYawError = 0f;
        var rows = new List<object>();

        void Reject(string name, Vector3 target, float yaw, ScaraJointLab.IkSelection mode)
        {
            var targets = Targets(lab);
            var current = Current(lab);
            var moving = MovingFlags(lab);
            Require(!lab.TryMoveTcp(target, yaw, mode, out _, out var error) && !string.IsNullOrEmpty(error), name + ": accepted");
            Require(Targets(lab) == targets && Current(lab) == current && MovingFlags(lab).SequenceEqual(moving), name + ": partial command or stop");
            rows.Add(new { name, passed = true, error });
            rejected++;
        }

        void Measure(string name)
        {
            float positionError = Vector3.Distance(lab.ActualTcpPosition, goal);
            float yawError = Mathf.Abs(Mathf.DeltaAngle(lab.ActualTcpYawDegrees, 0f));
            Require(positionError < .000002f && yawError < .001f, name + ": actual TCP mismatch");
            maxPositionError = Mathf.Max(maxPositionError, positionError);
            maxYawError = Mathf.Max(maxYawError, yawError);
            actualArrivals++;
            rows.Add(new { name, passed = true, positionErrorMetres = positionError, yawErrorDegrees = yawError });
        }

        try
        {
            lab.HomeAll();
            await Until(() => AtTargets(lab), token);
            await Until(() => move.enabledInHierarchy && candidates.text.Contains("후보 2개"), token);
            Require(lab.TrySelectTcpSolution(goal, 0f, ScaraJointLab.IkSelection.Nearest, out _, out var nearest, out _) && nearest.ElbowSign == 1, "Nearest at home");
            Require(lab.TryMoveTcp(goal, 0f, ScaraJointLab.IkSelection.PositiveElbow, out var positive, out _), "Positive command rejected");
            Require(Targets(lab) == Joints(positive.Joints) && Mathf.Abs(lab.Axis3.TargetPosition - .084823f) < .000001f, "Four command targets or mm-to-m goal mismatch");
            await Until(() => lab.Axis2.CurrentPosition > 3f, token);
            Reject("Unreachable while moving", new Vector3(.8f,-.2f,0f), 0f, ScaraJointLab.IkSelection.Nearest);
            Require(MovingFlags(lab).Any(flag => flag), "Invalid request stopped the existing move");
            while (!AtTargets(lab))
            {
                Require(lab.TryReadFkDiagnostics(out var sample, out _) && sample.PositionErrorMetres < .000002f && sample.YawErrorDegrees < .001f, "Moving FK regression");
                movingSamples++;
                if (movingSamples > 300) throw new TimeoutException("Positive arrival timeout");
                await Task.Delay(30, token);
            }
            Measure("Positive elbow actual arrival");
            selection.value = "q2 음수 해";
            await Until(() => chosen.text.Contains("q2 음수"), token);
            Require(lab.TryMoveTcp(goal, 0f, ScaraJointLab.IkSelection.NegativeElbow, out _, out _), "Negative command rejected");
            await Until(() => AtTargets(lab), token);
            Measure("Negative elbow same target actual arrival");
            Require(lab.TrySelectTcpSolution(goal, 0f, ScaraJointLab.IkSelection.Nearest, out _, out nearest, out _) && nearest.ElbowSign == -1, "Nearest must use current joints");

            Require(lab.TryMoveTcp(home, 0f, ScaraJointLab.IkSelection.Nearest, out _, out _), "Home singular goal rejected");
            await Until(() => lab.Axis2.CurrentPosition > -85f, token);
            lab.StopAll();
            var stopped = Current(lab);
            await Task.Delay(160, token);
            Require(Current(lab) == stopped && !MovingFlags(lab).Any(flag => flag), "Stop did not hold actual joints");
            rows.Add(new { name = "Intermediate stop", passed = true });

            Reject("Shoulder limit", new Vector3(-.7f,-.2f,0f), 180f, ScaraJointLab.IkSelection.Nearest);
            Reject("Slide limit", new Vector3(.4f,-.3f,-.3f), 0f, ScaraJointLab.IkSelection.Nearest);
            Reject("Invalid number", new Vector3(float.NaN,-.2f,-.3f), 0f, ScaraJointLab.IkSelection.Nearest);
            Reject("Invalid selection", goal, 0f, (ScaraJointLab.IkSelection)99);

            var commands = Targets(lab);
            x.value = 800f;
            await Until(() => !move.enabledInHierarchy && candidates.text == "후보 없음" && feedback.text.Contains("닿지"), token);
            Require(chosen.text == "선택 예정: 없음" && Targets(lab) == commands, "Invalid preview retained stale candidate or changed commands");
            x.value = 400f;
            SetFloat(lab.Axis2, "maximumDegrees", 0f);
            selection.value = "q2 양수 해";
            await Until(() => !move.enabledInHierarchy && feedback.text.Contains("선택한 굽힘"), token);
            Reject("Requested elbow branch unavailable", goal, 0f, ScaraJointLab.IkSelection.PositiveElbow);
            SetFloat(lab.Axis2, "maximumDegrees", maxAxis2);
            selection.value = "현재 자세에서 가까운 해";
            Require(lab.TrySelectTcpSolution(goal, 0f, ScaraJointLab.IkSelection.Nearest, out _, out _, out _), "Fresh preview rejected");
            SetFloat(lab, "fkLink1Metres", 0f);
            Reject("Geometry changed after preview", goal, 0f, ScaraJointLab.IkSelection.Nearest);
            await Until(() => candidates.text == "후보 없음" && chosen.text == "선택 예정: 없음", token);
            SetFloat(lab, "fkLink1Metres", link1);
            lab.Axis4.enabled = false;
            Reject("Fourth axis disabled before request", goal, 0f, ScaraJointLab.IkSelection.Nearest);
            lab.Axis4.enabled = axis4Enabled;

            EditorApplication.isPaused = true;
            await Task.Delay(220, token);
            Require(!move.enabledInHierarchy && !x.enabledInHierarchy, "Paused UI enabled");
            Reject("Editor paused", goal, 0f, ScaraJointLab.IkSelection.Nearest);
            EditorApplication.isPaused = false;
            Time.timeScale = 0f;
            Reject("Simulation time stopped", goal, 0f, ScaraJointLab.IkSelection.Nearest);
            Time.timeScale = originalTimeScale;
            await Until(() => move.enabledInHierarchy, token);
            lab.HomeAll();
            await Until(() => AtTargets(lab), token);

            var baseline = JObject.Parse(File.ReadAllText("Logs/LW06/controls-edit-mode.json"))["sourceHashes"].ToObject<Dictionary<string,string>>();
            bool unchanged = SourcePaths.All(path => baseline[path] == Hash(path));
            Require(unchanged, "Source scene, prefab or settings changed during Play verification");
            var report = new { passed = true, actualArrivals, rejectedRequests = rejected, movingSamples,
                maxActualPositionErrorMetres = maxPositionError, maxActualYawErrorDegrees = maxYawError,
                nearestUsesCurrentJoints = true, stoppedPoseHeld = true, previewFailureCleared = true,
                pausedControlsDisabled = true, sourceAssetsUnchanged = unchanged, directButtonClicks = 0,
                scope = "API-driven IK movement and UI state reads; manual button verification pending; not 100-goal validation or collision safety", rows };
            File.WriteAllText("Logs/LW06/controls-play-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;
        }
        finally
        {
            EditorApplication.isPaused = false;
            Time.timeScale = originalTimeScale;
            if (lab != null)
            {
                SetFloat(lab, "fkLink1Metres", link1);
                SetFloat(lab.Axis2, "maximumDegrees", maxAxis2);
                lab.Axis4.enabled = axis4Enabled;
                lab.StopAll();
            }
        }
    }

    private static Vector4 Targets(ScaraJointLab l) => new Vector4(l.Axis1.TargetPosition,l.Axis2.TargetPosition,l.Axis3.TargetPosition,l.Axis4.TargetPosition);
    private static Vector4 Current(ScaraJointLab l) => new Vector4(l.Axis1.CurrentPosition,l.Axis2.CurrentPosition,l.Axis3.CurrentPosition,l.Axis4.CurrentPosition);
    private static Vector4 Joints(ScaraInverseKinematics.JointValues j) => new Vector4(j.Q1Degrees,j.Q2Degrees,j.D3Metres,j.Q4Degrees);
    private static bool[] MovingFlags(ScaraJointLab l) => new[] { l.Axis1.IsMoving,l.Axis2.IsMoving,l.Axis3.IsMoving,l.Axis4.IsMoving };
    private static bool AtTargets(ScaraJointLab l) => !MovingFlags(l).Any(flag => flag) && l.Axis1.IsAtTarget && l.Axis2.IsAtTarget && l.Axis3.IsAtTarget && l.Axis4.IsAtTarget;
    private static void SetFloat(UnityEngine.Object target, string name, float value)
    {
        var settings = new SerializedObject(target);
        settings.FindProperty(name).floatValue = value;
        settings.ApplyModifiedPropertiesWithoutUndo();
    }
    private static async Task Until(Func<bool> condition, CancellationToken token)
    {
        var watch = Stopwatch.StartNew();
        while (!condition())
        {
            token.ThrowIfCancellationRequested();
            if (EditorApplication.isPaused) throw new InvalidOperationException("Verification interrupted by user pause");
            if (watch.Elapsed.TotalSeconds > 12) throw new TimeoutException("IK control verification timeout");
            await Task.Delay(40, token);
        }
    }
    private static void Require(bool value, string message) { if (!value) throw new InvalidOperationException(message); }
    private static string Hash(string path)
    {
        using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", "");
    }
}
