using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

public static class VerifyScaraFkDiagnostics
{
    public static async Task<object> Run()
    {
        if (!EditorApplication.isPlaying || EditorApplication.isPaused)
            throw new InvalidOperationException("Run RobotLab in unpaused Play Mode.");
        var lab = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        Require(lab != null && lab.IsReady, "SCARA controls must be ready.");
        var token = lab.destroyCancellationToken;
        var settings = new SerializedObject(lab);
        float originalLink1 = settings.FindProperty("fkLink1Metres").floatValue;
        float originalHeight12 = settings.FindProperty("fkHeight12Metres").floatValue;
        Require(EditorApplication.ExecuteMenuItem("LineWatch/SCARA 4축 조작 테스트"), "SCARA menu did not open.");
        var window = EditorWindow.GetWindow<ScaraJointLabWindow>();
        var calculatedLabel = window.rootVisualElement.Q<Label>("scara-fk-calculated");
        var actualLabel = window.rootVisualElement.Q<Label>("scara-fk-actual");
        var errorLabel = window.rootVisualElement.Q<Label>("scara-fk-errors");
        var failure = window.rootVisualElement.Q<HelpBox>("scara-fk-failure");
        Require(calculatedLabel != null && actualLabel != null && errorLabel != null && failure != null,
            "The FK diagnostic display is incomplete.");
        int samples = 0;
        float maxPositionError = 0f, maxYawError = 0f;
        try
        {
            lab.HomeAll();
            await Until(() => AllHome(lab), token);
            await Until(() => calculatedLabel.text.Contains("계산 TCP") && actualLabel.text.Contains("실제 TCP") &&
                errorLabel.text.Contains("위치 오차"), token);
            var home = ReadValid(lab);
            var geometry = new ScaraForwardKinematics.Geometry(0.325f, 0.375f, 0.016783f, -0.097783f,
                new Vector3(0f, -0.034177f, 0f));
            var goal = ScaraForwardKinematics.Calculate(20f, -20f, 0.05f, 30f, geometry);
            lab.MoveJoint(0, 20f); lab.MoveJoint(1, -20f); lab.MoveJoint(2, 0.05f); lab.MoveJoint(3, 30f);
            var justCommanded = ReadValid(lab);
            float targetGap = Vector3.Distance(justCommanded.Calculated.PositionMetres, goal.PositionMetres);
            Require(targetGap > 0.01f && Vector3.Distance(justCommanded.Calculated.PositionMetres, home.Calculated.PositionMetres) < 0.000002f,
                "Diagnostics used command targets instead of current joint positions.");
            var watch = Stopwatch.StartNew();
            while (!AtTargets(lab))
            {
                token.ThrowIfCancellationRequested();
                if (EditorApplication.isPaused) throw new InvalidOperationException("Verification interrupted by Editor pause.");
                var sample = ReadValid(lab);
                samples++;
                maxPositionError = Mathf.Max(maxPositionError, sample.PositionErrorMetres);
                maxYawError = Mathf.Max(maxYawError, sample.YawErrorDegrees);
                if (watch.Elapsed.TotalSeconds > 7) throw new TimeoutException("Joint motion did not finish.");
                await Task.Delay(40, token);
            }
            var arrived = ReadValid(lab);
            Require(samples >= 3 && Vector3.Distance(arrived.Calculated.PositionMetres, home.Calculated.PositionMetres) > 0.01f,
                "The calculated TCP did not refresh during movement.");
            lab.HomeAll();
            await Until(() => AllHome(lab), token);

            // Perturb only the runtime FK input; do not move the model or save overrides.
            SetFloat(lab, "fkHeight12Metres", originalHeight12 + 0.01f);
            Require(lab.TryReadFkDiagnostics(out var mismatched, out _), "Finite offset input was rejected.");
            Require(Mathf.Abs(mismatched.PositionErrorMetres - 0.01f) < 0.000002f,
                "A 10mm geometry mismatch was not reported.");
            await Until(() => ReadDisplayedErrorMm(errorLabel.text) > 9.99f, token);
            Require(calculatedLabel.text != actualLabel.text && actualLabel.text.Contains("실제 TCP"),
                "Calculated and measured diagnostics were not displayed separately.");
            float mismatchDisplayedMm = ReadDisplayedErrorMm(errorLabel.text);
            Require(Mathf.Abs(mismatchDisplayedMm - 10f) < 0.01f, "Diagnostic label has the wrong position-error unit.");
            Require(Mathf.Abs(ReadDisplayedY(calculatedLabel.text) - ReadDisplayedY(actualLabel.text) - 10f) < 0.02f,
                "The UI did not show different calculated and measured positions for the geometry mismatch.");
            SetFloat(lab, "fkHeight12Metres", originalHeight12);
            SetFloat(lab, "fkLink1Metres", 0f);
            Require(!lab.TryReadFkDiagnostics(out _, out var invalidReason) && !string.IsNullOrEmpty(invalidReason),
                "Invalid geometry must return an explicit failure.");
            await Until(() => calculatedLabel.text.Contains("계산 불가") && errorLabel.text.Contains("계산 불가") &&
                failure.style.display.value == DisplayStyle.Flex, token);
            Require(actualLabel.text.Contains("실제 TCP") && !actualLabel.text.Contains("계산 불가"),
                "Actual TCP should remain visible when FK input is invalid.");
            SetFloat(lab, "fkLink1Metres", originalLink1);
            await Until(() => errorLabel.text.Contains("위치 오차") && failure.style.display.value == DisplayStyle.None, token);
            var restored = ReadValid(lab);
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"), movingSamples = samples,
                maxMovingPositionErrorMetres = maxPositionError, maxMovingYawErrorDegrees = maxYawError,
                targetVsCurrentGapMetres = targetGap, mismatchDisplayedMm,
                invalidGeometryShown = true, measuredTcpRetainedOnFailure = true, recoveryPassed = true,
                restoredHomeErrorMetres = restored.PositionErrorMetres, manualClicksPerformed = false,
                scope = "LW-05c current-value sampling and UI text/status binding; full rig/tool variation validation deferred" };
            Directory.CreateDirectory("Logs/LW05");
            File.WriteAllText("Logs/LW05/diagnostics-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;
        }
        finally
        {
            if (lab != null && Application.isPlaying)
            {
                SetFloat(lab, "fkLink1Metres", originalLink1);
                SetFloat(lab, "fkHeight12Metres", originalHeight12);
                lab.StopAll();
            }
        }
    }

    private static ScaraJointLab.FkDiagnostics ReadValid(ScaraJointLab lab)
    {
        Require(lab.TryReadFkDiagnostics(out var sample, out var error), "FK diagnostics failed: " + error);
        Require(sample.PositionErrorMetres < 0.000002f && sample.YawErrorDegrees < 0.001f,
            "Current-joint FK and actual TCP disagree.");
        return sample;
    }
    private static float ReadDisplayedErrorMm(string text)
    {
        const string prefix = "위치 오차 ";
        if (!text.StartsWith(prefix)) return -1f;
        string value = text.Substring(prefix.Length).Split(' ')[0];
        return float.TryParse(value, NumberStyles.Float, CultureInfo.CurrentCulture, out var parsed) ? parsed : -1f;
    }
    private static float ReadDisplayedY(string text)
    {
        var segments = text.Split('/');
        if (segments.Length < 2) return float.NaN;
        string value = segments[1].Trim().Substring(2).Trim();
        return float.TryParse(value, NumberStyles.Float, CultureInfo.CurrentCulture, out var parsed) ? parsed : float.NaN;
    }
    private static void SetFloat(ScaraJointLab lab, string name, float value)
    {
        var settings = new SerializedObject(lab);
        settings.FindProperty(name).floatValue = value;
        settings.ApplyModifiedPropertiesWithoutUndo();
    }
    private static bool AllHome(ScaraJointLab lab) => !Moving(lab) && lab.Axis1.IsAtTarget && lab.Axis2.IsAtTarget &&
        lab.Axis3.IsAtTarget && lab.Axis4.IsAtTarget && Mathf.Abs(lab.Axis1.CurrentPosition) < 0.001f &&
        Mathf.Abs(lab.Axis2.CurrentPosition) < 0.001f && lab.Axis3.CurrentPosition < 0.000001f &&
        Mathf.Abs(lab.Axis4.CurrentPosition) < 0.001f;
    private static bool AtTargets(ScaraJointLab lab) => !Moving(lab) && lab.Axis1.IsAtTarget && lab.Axis2.IsAtTarget &&
        lab.Axis3.IsAtTarget && lab.Axis4.IsAtTarget;
    private static bool Moving(ScaraJointLab lab) => lab.Axis1.IsMoving || lab.Axis2.IsMoving || lab.Axis3.IsMoving || lab.Axis4.IsMoving;
    private static async Task Until(Func<bool> condition, CancellationToken token)
    {
        var watch = Stopwatch.StartNew();
        while (!condition())
        {
            token.ThrowIfCancellationRequested();
            if (EditorApplication.isPaused) throw new InvalidOperationException("Verification interrupted by Editor pause.");
            if (watch.Elapsed.TotalSeconds > 7) throw new TimeoutException("Diagnostic display did not reach the expected state.");
            await Task.Delay(40, token);
        }
    }
    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
