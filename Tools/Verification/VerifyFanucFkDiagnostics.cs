using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

public static class VerifyFanucFkDiagnostics
{
    public static async Task<object> Run()
    {
        if (!EditorApplication.isPlaying || EditorApplication.isPaused)
            throw new InvalidOperationException("Run RobotLab in unpaused Play Mode.");
        var lab = UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        Require(lab != null && lab.IsReady, "FANUC is not ready.");
        var token = lab.destroyCancellationToken;
        var paths = new[] { "Assets/01.Scenes/RobotLab.unity", "Assets/03.Prefabs/FanucRobot_Work.prefab",
            "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };
        var hashes = paths.ToDictionary(p => p, Hash);
        var settings = new SerializedObject(lab);
        var tool = settings.FindProperty("geometry").FindPropertyRelative("toolPositionMetres").vector3Value;
        var toolRotation = settings.FindProperty("geometry").FindPropertyRelative("toolRotation").quaternionValue;
        var frame = lab.RobotFrame;
        Require(EditorApplication.ExecuteMenuItem("LineWatch/FANUC 조작 테스트"), "FANUC menu did not open.");
        var window = EditorWindow.GetWindow<FanucJointLabWindow>();
        var calculated = window.rootVisualElement.Q<Label>("fanuc-fk-calculated");
        var actual = window.rootVisualElement.Q<Label>("fanuc-fk-actual");
        var errors = window.rootVisualElement.Q<Label>("fanuc-fk-errors");
        var failure = window.rootVisualElement.Q<HelpBox>("fanuc-fk-failure");
        Require(calculated != null && actual != null && errors != null && failure != null, "Missing diagnostic display.");
        int samples = 0;
        float maxPosition = 0, maxRotation = 0;
        try
        {
            lab.HomeAll();
            await Until(() => At(lab, new float[6]), token);
            await Until(() => errors.text.StartsWith("위치 오차 ") && actual.text.Contains("방향 q(x,y,z,w)"), token);
            var home = ReadValid(lab);
            var target = new[] { 12f, -18f, 16f, -10f, 14f, -20f };
            var goal = FanucForwardKinematics.Calculate(target, lab.Geometry);
            for (int i = 0; i < 6; i++) Require(lab.MoveJoint(i, target[i]), "Joint command was rejected.");
            var justCommanded = ReadValid(lab);
            float targetGap = Vector3.Distance(justCommanded.Calculated.PositionMetres, goal.PositionMetres);
            Require(targetGap > .01f && Vector3.Distance(justCommanded.Calculated.PositionMetres,
                home.Calculated.PositionMetres) < .000002f, "Diagnostics read target values instead of current joints.");
            var watch = Stopwatch.StartNew();
            while (!At(lab, target))
            {
                token.ThrowIfCancellationRequested();
                Require(!EditorApplication.isPaused, "Verification interrupted by pause.");
                var sample = ReadValid(lab);
                samples++;
                maxPosition = Mathf.Max(maxPosition, sample.PositionErrorMetres);
                maxRotation = Mathf.Max(maxRotation, sample.RotationErrorDegrees);
                Require(watch.Elapsed.TotalSeconds < 8, "FANUC movement timed out.");
                await Task.Delay(40, token);
            }
            Require(samples >= 3, "No meaningful in-flight samples.");
            var arrived = ReadValid(lab);
            await Until(() => calculated.text == Format("계산 TCP", arrived.Calculated.PositionMetres, arrived.Calculated.Rotation)
                && actual.text == Format("실제 TCP", arrived.ActualPositionMetres, arrived.ActualRotation), token);
            lab.HomeAll();
            await Until(() => At(lab, new float[6]), token);
            Require(lab.MoveJoint(5, 15), "A6 command was rejected.");
            await Until(() => !lab.Joints[5].IsMoving && lab.Joints[5].IsAtTarget, token);
            var wrist = ReadValid(lab);
            float wristPositionChange = Vector3.Distance(home.ActualPositionMetres, wrist.ActualPositionMetres);
            float wristRotationChange = Quaternion.Angle(home.ActualRotation, wrist.ActualRotation);
            Require(wristPositionChange < .000002f && Mathf.Abs(wristRotationChange - 15) < .001f,
                "A6 should change full orientation while this TCP stays on its axis.");
            lab.HomeAll();
            await Until(() => At(lab, new float[6]), token);
            await Until(() => actual.text == Format("실제 TCP", home.ActualPositionMetres, home.ActualRotation), token);
            string actualHomeText = actual.text;
            string commandState = State(lab);

            Set(lab, "toolPositionMetres", p => p.vector3Value = tool + new Vector3(0,.01f,0));
            Require(lab.TryReadFkDiagnostics(out var mismatch, out _)
                && Mathf.Abs(mismatch.PositionErrorMetres - .01f) < .000002f, "Missing 10mm input mismatch.");
            await Until(() => ReadError(errors.text, 0) > 9.99f, token);
            float displayedMm = ReadError(errors.text, 0);
            Require(Mathf.Abs(displayedMm-10) < .01f && actual.text == actualHomeText,
                "Wrong UI position unit or actual TCP changed with calculated geometry.");
            Set(lab, "toolPositionMetres", p => p.vector3Value = tool);
            Set(lab, "toolRotation", p => p.quaternionValue = toolRotation * Quaternion.AngleAxis(10,Vector3.up));
            Require(lab.TryReadFkDiagnostics(out mismatch, out _) && Mathf.Abs(mismatch.RotationErrorDegrees-10) < .001f,
                "Missing 10-degree full rotation mismatch.");
            await Until(() => ReadError(errors.text, 1) > 9.99f, token);
            float displayedDegrees = ReadError(errors.text, 1);
            Require(Mathf.Abs(displayedDegrees-10) < .01f && actual.text == actualHomeText, "Wrong full rotation display.");

            Set(lab, "toolRotation", p => p.quaternionValue = default);
            Require(!lab.TryReadFkDiagnostics(out _, out var reason) && !string.IsNullOrEmpty(reason), "Invalid rotation accepted.");
            await Until(() => calculated.text.Contains("계산 불가") && errors.text.Contains("계산 불가")
                && failure.style.display.value == DisplayStyle.Flex, token);
            Require(actual.text == actualHomeText && State(lab) == commandState, "FK failure changed actual pose or commands.");
            Set(lab, "toolRotation", p => p.quaternionValue = toolRotation);
            await Until(() => errors.text.StartsWith("위치 오차 ") && failure.style.display.value == DisplayStyle.None, token);

            SetFrame(lab, null);
            Require(!lab.TryReadFkDiagnostics(out _, out _) && !lab.TryReadActualTcp(out _,out _,out _), "Missing frame accepted.");
            await Until(() => actual.text.Contains("읽기 불가") && calculated.text.Contains("계산 불가"), token);
            SetFrame(lab, frame);
            await Until(() => actual.text == actualHomeText && failure.style.display.value == DisplayStyle.None, token);
            Require(State(lab) == commandState, "Diagnostics changed motor commands.");
            var restored = ReadValid(lab);
            Require(hashes.All(p => Hash(p.Key) == p.Value), "A source asset was changed.");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"), movingSamples = samples,
                maxMovingPositionErrorMetres = maxPosition, maxMovingRotationErrorDegrees = maxRotation,
                targetVsCurrentGapMetres = targetGap, wristPositionChangeMetres = wristPositionChange,
                wristRotationChangeDegrees = wristRotationChange, mismatchDisplayedMm = displayedMm,
                mismatchDisplayedDegrees = displayedDegrees, invalidGeometryShown = true,
                actualRetainedOnGeometryFailure = true, missingFrameShown = true, recoveryPassed = true,
                diagnosticsPreservedCommands = true, restoredHomeErrorMetres = restored.PositionErrorMetres,
                manualClicksPerformed = false, sourceAssetsUnchanged = true, sourceHashes = hashes,
                scope = "LW-07B process 2: API/UI diagnostics; user clicks and 100-pose/placement/tool campaign remain separate" };
            Directory.CreateDirectory("Logs/LW07B");
            File.WriteAllText("Logs/LW07B/diagnostics-verification.json", JsonConvert.SerializeObject(report,Formatting.Indented));
            return report;
        }
        finally
        {
            if (lab != null && Application.isPlaying)
            {
                Set(lab, "toolPositionMetres", p => p.vector3Value = tool);
                Set(lab, "toolRotation", p => p.quaternionValue = toolRotation);
                SetFrame(lab, frame);
                lab.StopAll();
            }
        }
    }

    private static FanucJointLab.FkDiagnostics ReadValid(FanucJointLab lab)
    {
        Require(lab.TryReadFkDiagnostics(out var sample, out var error), "FK diagnostic failure: " + error);
        Require(sample.PositionErrorMetres < .000002f && sample.RotationErrorDegrees < .001f, "Model/FK mismatch.");
        return sample;
    }
    private static string Format(string label, Vector3 p, Quaternion r) =>
        $"{label} · RobotFrame\n위치 X {p.x*1000f:F3} · Y {p.y*1000f:F3} · Z {p.z*1000f:F3} mm\n"
        + $"방향 q(x,y,z,w) = ({r.x:F6}, {r.y:F6}, {r.z:F6}, {r.w:F6})";
    private static float ReadError(string text, int line)
    {
        var lines = text.Split('\n');
        if (lines.Length <= line || !lines[line].StartsWith(line == 0 ? "위치 오차 " : "전체 회전 오차 ")) return -1;
        string number = lines[line].Substring(line == 0 ? "위치 오차 ".Length : "전체 회전 오차 ".Length).Split(' ')[0].TrimEnd('°');
        return float.TryParse(number,NumberStyles.Float,CultureInfo.CurrentCulture,out float value) ? value : -1;
    }
    private static void Set(FanucJointLab lab, string field, Action<SerializedProperty> assign)
    {
        var settings = new SerializedObject(lab);
        assign(settings.FindProperty("geometry").FindPropertyRelative(field));
        settings.ApplyModifiedPropertiesWithoutUndo();
    }
    private static void SetFrame(FanucJointLab lab, Transform value)
    {
        var settings = new SerializedObject(lab);
        settings.FindProperty("robotFrame").objectReferenceValue = value;
        settings.ApplyModifiedPropertiesWithoutUndo();
    }
    private static bool At(FanucJointLab lab,float[] q) => lab.Joints.Select((j,i) => !j.IsMoving && Mathf.Abs(j.CurrentPosition-q[i]) < .001f).All(v => v);
    private static string State(FanucJointLab lab) => JsonConvert.SerializeObject(lab.Joints.Select(j => new {j.CurrentPosition,j.TargetPosition,j.IsMoving}));
    private static async Task Until(Func<bool> condition,CancellationToken token)
    {
        var watch = Stopwatch.StartNew();
        while (!condition())
        {
            token.ThrowIfCancellationRequested();
            Require(!EditorApplication.isPaused && watch.Elapsed.TotalSeconds < 8, "UI/movement wait timed out or paused.");
            await Task.Delay(40,token);
        }
    }
    private static string Hash(string path) { using(var sha=SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Require(bool condition,string message) { if(!condition) throw new InvalidOperationException(message); }
}
