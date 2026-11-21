using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

public static class VerifyManualTcpUi
{
    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); var s = l.Scara; var f = l.Fanuc;
        await Until(() => t.Count == 1, "Initial registration timeout.");
        string products = Products();
        var window = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); window.Show(); window.RefreshCell();
        var selection = window.EquipmentSelection; var view = window.rootVisualElement.Q<CellEquipmentView>();
        var manual = window.rootVisualElement.Q<CellManualTcpView>(); var command = manual.Command;
        var move = manual.Q<Button>("equipment-manual-move");
        Require(!manual.Q<Foldout>("equipment-manual-panel").value, "Manual inputs clutter the default view.");
        Require(selection.TrySelect(l.ScaraId, out var error), error); view.Refresh();
        uint sv = s.CommandVersion, fv = f.CommandVersion;
        Require(command.TryLoadCurrent(out error), error); view.Refresh();
        Require(manual.Q<FloatField>("equipment-manual-yaw").style.display.value == DisplayStyle.Flex && manual.Q<FloatField>("equipment-manual-rx").style.display.value == DisplayStyle.None,
            "SCARA rotation inputs are not yaw-only.");
        var center = l.ScaraFootprint.transform.position;
        var scaraGoal = s.RobotFrame.InverseTransformPoint(new Vector3(center.x + .25f, s.Tcp.position.y - .003f, center.z));
        command.SetDraft(scaraGoal * 1000, new Vector3(0, s.ActualTcpYawDegrees, 0)); view.Refresh();
        Require(command.CanExecute(out error) && move.enabledSelf && s.CommandVersion == sv && f.CommandVersion == fv, error ?? "Load/input issued a move.");
        Require(selection.TrySelect(l.FanucId, out error) && selection.TrySelect(l.ScaraId, out error), error);
        Require(!command.TryExecute(out _) && s.CommandVersion == sv && f.CommandVersion == fv, "Round-trip selection reused an old draft.");
        view.Refresh(); Require(!command.HasDraft && !move.enabledSelf, "Selection did not clear draft/UI eligibility.");

        Require(selection.TrySelect(l.FanucId, out error), error); view.Refresh();
        Require(command.TryLoadCurrent(out error), error); view.Refresh();
        Require(f.TryReadActualTcp(out var actual, out _, out error) && Vector3.Distance(command.PositionMillimetres, actual * 1000) < .001f &&
            manual.Q<FloatField>("equipment-manual-yaw").style.display.value == DisplayStyle.None && manual.Q<FloatField>("equipment-manual-rz").style.display.value == DisplayStyle.Flex &&
            s.CommandVersion == sv && f.CommandVersion == fv, "FANUC load/mm/Euler fields or read-only behavior mismatch.");
        command.SetDraft(new Vector3(float.NaN, 0, 0), Vector3.zero); view.Refresh();
        Require(!move.enabledSelf && !command.TryExecute(out error) && error.Contains("유한"), "Nonfinite draft was accepted.");
        command.SetDraft(f.RobotFrame.InverseTransformPoint(new Vector3(10, 1, 10)) * 1000, Vector3.zero); view.Refresh();
        Require(!move.enabledSelf && !command.TryExecute(out error) && error.Contains("작업구역") && f.CommandVersion == fv, "Outside-zone draft bypassed admission.");
        var joints = f.Joints.Select(j => j.CurrentPosition).ToArray(); joints[0] += .5f;
        var goal = FanucForwardKinematics.Calculate(joints, f.Geometry);
        command.SetDraft(goal.PositionMetres * 1000, goal.Rotation.eulerAngles); view.Refresh();
        Require(move.enabledSelf && Mathf.Abs(manual.Q<FloatField>("equipment-manual-x").value - command.PositionMillimetres.x) < .001f &&
            manual.Q<HelpBox>("equipment-manual-eligibility").text.Contains("실행 시"), "Valid draft UI/final validation guidance missing.");
        Require(command.TryExecute(out error), error); view.Refresh();
        Require(f.LastAcceptedTcpTarget.HasValue && Vector3.Distance(f.LastAcceptedTcpTarget.Value.position, goal.PositionMetres) < .000001f &&
            Quaternion.Angle(f.LastAcceptedTcpTarget.Value.rotation, goal.Rotation) < .1f && manual.Q<HelpBox>("equipment-manual-result").text.Contains("수락"),
            "Draft unit/rotation conversion or accepted result mismatch.");
        await Until(() => !f.IsTcpMoving, "Manual domain move timeout.");
        Require(f.TryReadActualTcp(out actual, out var rotation, out error) && Vector3.Distance(actual, goal.PositionMetres) < .002f && Quaternion.Angle(rotation, goal.Rotation) < .5f, error ?? "Actual goal mismatch.");
        Require(c.TryExecute(CellRunCommand.Start, out error), error); view.Refresh();
        Require(!move.enabledSelf && !command.TryExecute(out error) && error.Contains("수동 로봇 명령"), "Running cell allowed manual UI command.");
        Require(c.TryExecute(CellRunCommand.Stop, out error), error);
        Require(selection.TrySelect(CellEquipmentSelection.HandoffId, out error), error); view.Refresh();
        Require(manual.style.display.value == DisplayStyle.None && !command.HasDraft && !command.TryExecute(out _), "Handoff retained a runnable robot draft.");
        Require(!c.IsFaulted && Products() == products, "Manual UI domain flow changed products/faulted cell.");
        var report = new { passed = true, domainMoves = 1, directClicks = 0, callbacksOrUiInputInjected = 0,
            loadAndDraftDoNotMove = true, scaraYawAndFanucEulerFields = true, mmAndEulerConversionCorrect = true,
            selectionRoundTripInvalidatesDraft = true, nonfiniteAndOutsideRejected = true, runningRejected = true,
            acceptedAndActualTargetMatched = true, handoffHidesAndClearsDraft = true, productsPreserved = true };
        Directory.CreateDirectory("Logs/LW15"); File.WriteAllText("Logs/LW15/manual-tcp-ui.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;

        string Products() => JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(),
            slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            items = l.GetComponentsInChildren<GrippableItem>(true).Select(i => new { i.ProductId, owner = i.Owner != null ? i.Owner.GetInstanceID() : 0,
                position = i.Body.position.ToString("R"), rotation = i.Body.rotation.ToString("R") }) });
    }
    private static async Task Until(Func<bool> predicate, string message)
    { var watch = Stopwatch.StartNew(); while (!predicate()) { Require(watch.Elapsed.TotalSeconds < 10, message); await Task.Delay(1); } }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
