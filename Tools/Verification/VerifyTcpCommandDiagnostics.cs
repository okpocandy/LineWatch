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

public static class VerifyTcpCommandDiagnostics
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
        var model = window.EquipmentSelection; var view = window.rootVisualElement.Q<CellEquipmentView>();
        Require(!s.LastAcceptedTcpPosition.HasValue && !f.LastAcceptedTcpTarget.HasValue, "Fresh session has accepted goals.");
        Require(s.TryReadFkDiagnostics(out var sd, out var error), error);
        var zoneCenter = l.ScaraFootprint.transform.position;
        var sg = s.RobotFrame.InverseTransformPoint(new Vector3(zoneCenter.x + .25f, s.Tcp.position.y - .003f, zoneCenter.z));
        Require(s.TryMoveTcp(sg, sd.ActualYawDegrees, ScaraJointLab.IkSelection.Nearest, out _, out error), error);
        Require(s.LastTcpRequestAccepted == true && s.LastMoveIkResult != null && s.LastMoveIkResult.Success && s.LastAcceptedTcpPosition.Value == sg, "SCARA accepted snapshot mismatch.");
        Read(l.ScaraId);
        Require(view.Q<Label>("equipment-diagnostic-target-error").text.Contains("수평각") && view.Q<Label>("equipment-diagnostic-ik").text.Contains("해석해"), "SCARA target/analytical result missing.");
        await Until(() => !s.IsTrajectoryMoving, "SCARA motion timeout.");
        Require(s.TryReadFkDiagnostics(out sd, out error) && Vector3.Distance(sg, sd.ActualPositionMetres) < .002f, error ?? "SCARA target mismatch.");
        uint sv = s.CommandVersion; var st = new[] { s.Axis1.TargetPosition, s.Axis2.TargetPosition, s.Axis3.TargetPosition, s.Axis4.TargetPosition };
        var scaraOutsideReach = s.RobotFrame.InverseTransformPoint(s.RobotFrame.TransformPoint(sg) + Vector3.up * 1000);
        Require(!s.TryMoveTcp(scaraOutsideReach, sd.ActualYawDegrees, ScaraJointLab.IkSelection.Nearest, out _, out error) &&
            s.LastMoveIkResult != null && !s.LastMoveIkResult.Success && s.LastTcpRequestAccepted == false && s.LastAcceptedTcpPosition.Value == sg && s.CommandVersion == sv &&
            st.SequenceEqual(new[] { s.Axis1.TargetPosition, s.Axis2.TargetPosition, s.Axis3.TargetPosition, s.Axis4.TargetPosition }), "SCARA rejected request changed accepted target/motion.");
        Read(l.ScaraId);
        Require(view.Q<Label>("equipment-diagnostic-request").text.Contains(error) && view.Q<Label>("equipment-diagnostic-ik").text.Contains("실패"), "SCARA failure explanation missing.");

        var joints = f.Joints.Select(j => j.CurrentPosition).ToArray(); joints[0] += .5f;
        var fg = FanucForwardKinematics.Calculate(joints, f.Geometry);
        Require(f.TryMoveTcp(fg.PositionMetres, fg.Rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out var preview, out error), error);
        Require(f.LastTcpRequestAccepted == true && f.LastMoveIkPreview.HasValue && ReferenceEquals(f.LastMoveIkPreview.Value.Result, preview.Result) &&
            f.LastAcceptedTcpTarget.Value.position == fg.PositionMetres, "FANUC accepted snapshot mismatch.");
        Read(l.FanucId);
        Require(view.Q<Label>("equipment-diagnostic-target-error").text.Contains("전체 회전") && view.Q<Label>("equipment-diagnostic-ik").text.Contains("수렴"), "FANUC whole rotation/convergence display missing.");
        await Until(() => !f.IsTcpMoving, "FANUC motion timeout.");
        Require(f.TryReadFkDiagnostics(out var fd, out error) && Vector3.Distance(fg.PositionMetres, fd.ActualPositionMetres) < .002f && Quaternion.Angle(fg.Rotation, fd.ActualRotation) < .5f,
            error ?? "FANUC actual target mismatch.");
        uint fv = f.CommandVersion; var ft = f.Joints.Select(j => j.TargetPosition).ToArray(); var accepted = f.LastAcceptedTcpTarget.Value;
        var fanucOutsideReach = f.RobotFrame.InverseTransformPoint(f.RobotFrame.TransformPoint(fg.PositionMetres) + Vector3.up * 1000);
        Require(!f.TryMoveTcp(fanucOutsideReach, fg.Rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error) &&
            f.LastMoveIkPreview.HasValue && !f.LastMoveIkPreview.Value.Result.Succeeded && f.LastTcpRequestAccepted == false && f.LastAcceptedTcpTarget.Value.Equals(accepted) &&
            f.CommandVersion == fv && ft.SequenceEqual(f.Joints.Select(j => j.TargetPosition)), "FANUC rejected request changed accepted target/motion.");
        Read(l.FanucId);
        Require(view.Q<Label>("equipment-diagnostic-request").text.Contains(error) && view.Q<Label>("equipment-diagnostic-ik").text.Contains("실패"), "FANUC failure explanation missing.");
        Require(c.TryExecute(CellRunCommand.Start, out error), error);
        Require(!s.TryMoveTcp(sg, sd.ActualYawDegrees, ScaraJointLab.IkSelection.Nearest, out _, out error) && s.LastMoveIkResult == null && s.LastAcceptedTcpPosition.Value == sg,
            "Admission rejection was represented as an IK solve.");
        Read(l.ScaraId);
        Require(view.Q<Label>("equipment-diagnostic-request").text.Contains("IK 계산 전") && view.Q<Label>("equipment-diagnostic-ik").text.Contains("계산 기록 없음"), "Pre-IK rejection display missing.");
        Require(c.TryExecute(CellRunCommand.Stop, out error), error);
        Require(Products() == products && !c.IsFaulted, "Diagnostic command tests changed products or faulted the cell.");
        var report = new { passed = true, normalMoves = 2, unreachableRejected = 2, preIkAdmissionRejected = 1, acceptedGoalsPreservedOnRejection = true,
            actualTargetsReached = true, cachedResultsReadWithoutResolve = true, productsPreserved = true, directClicks = 0, callbacksOrUiInputInjected = 0 };
        Directory.CreateDirectory("Logs/LW15"); File.WriteAllText("Logs/LW15/tcp-command-diagnostics.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return report;

        void Read(string id)
        {
            Require(model.TrySelect(id, out var e), e);
            uint beforeS = s.CommandVersion, beforeF = f.CommandVersion;
            var beforeSi = s.LastMoveIkResult; var beforeFi = f.LastMoveIkPreview;
            view.Refresh(); view.Refresh();
            Require(s.CommandVersion == beforeS && f.CommandVersion == beforeF && ReferenceEquals(s.LastMoveIkResult, beforeSi) &&
                (f.LastMoveIkPreview.HasValue == beforeFi.HasValue) && (!beforeFi.HasValue || ReferenceEquals(f.LastMoveIkPreview.Value.Result, beforeFi.Value.Result)), "View issued motion or recalculated IK.");
        }
        string Products() => JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(),
            slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            items = l.GetComponentsInChildren<GrippableItem>(true).Select(i => new { i.ProductId, owner = i.Owner != null ? i.Owner.GetInstanceID() : 0,
                position = i.Body.position.ToString("R"), rotation = i.Body.rotation.ToString("R") }) });
    }
    private static async Task Until(Func<bool> predicate, string message)
    { var watch = Stopwatch.StartNew(); while (!predicate()) { Require(watch.Elapsed.TotalSeconds < 10, message); await Task.Delay(1); } }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}

public static class VerifyManualTcpAdmission
{
    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh LineWatch Play.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Use LineWatch.");
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); var s = l.Scara; var f = l.Fanuc;
        var sg = s.GetComponent<RobotGripper>(); float scale = Time.timeScale; GrippableItem fixture = null;
        try
        {
            Time.timeScale = 5;
            // Reuse the changed goal-record test once, including in-zone but unreachable heights.
            await VerifyTcpCommandDiagnostics.Run();
            var ownS = s.RobotFrame.InverseTransformPoint(s.Tcp.position);
            var ownF = f.RobotFrame.InverseTransformPoint(f.Tcp.position);
            var sharedS = s.RobotFrame.InverseTransformPoint(l.HandoffPoint.position);
            var sharedF = f.RobotFrame.InverseTransformPoint(l.HandoffPoint.position);
            string beforeQueries = State();
            Require(c.CanMoveManualTcp(s, ownS, out var error) && c.CanMoveManualTcp(f, ownF, out error) &&
                c.CanMoveManualTcp(s, sharedS, out error) && c.CanMoveManualTcp(f, sharedF, out error), error);
            Require(State() == beforeQueries, "Eligibility query changed state.");
            var outside = s.RobotFrame.InverseTransformPoint(l.Fanuc.Tcp.position + Vector3.forward * -2);
            Reject(s, outside, "작업구역"); Reject(f, f.RobotFrame.InverseTransformPoint(s.RobotFrame.TransformPoint(outside)), "작업구역");
            Require(c.TryExecute(CellRunCommand.Start, out error) && c.TryExecute(CellRunCommand.Pause, out error), error);
            Reject(s, ownS, "수동 로봇 명령");
            Require(c.TryExecute(CellRunCommand.Stop, out error), error);
            l.HandoffZone.gameObject.SetActive(false);
            try { Reject(f, sharedF, "비활성"); } finally { l.HandoffZone.gameObject.SetActive(true); }

            fixture = UnityEngine.Object.Instantiate(l.ProductPrefab);
            var point = fixture.GetGripPoint(RobotWaypointSet.RobotType.Scara);
            var localOffset = Quaternion.Inverse(fixture.Body.rotation) * (point.position - fixture.Body.position);
            var localRotation = Quaternion.Inverse(fixture.Body.rotation) * point.rotation;
            var rotation = sg.Contact.rotation * Quaternion.Inverse(localRotation);
            fixture.Body.position = sg.Contact.position - rotation * localOffset; fixture.Body.rotation = rotation;
            await Task.Delay(40);
            Require(sg.TryGrip(fixture, out error), error);
            Reject(s, ownS, "두 집게"); Reject(f, ownF, "두 집게");
            Require(fixture.Owner == sg && sg.HeldItem == fixture, "Rejected motion changed actual grip ownership.");
            Require(sg.TryRelease(out error), error);
            UnityEngine.Object.Destroy(fixture.gameObject); fixture = null;

            // The peer robot can occupy the shared area without holding a product.
            var returnF = f.LastAcceptedTcpTarget.Value;
            var approach = l.FanucHandoffWaypoints.Approach;
            Require(f.TryMoveTcp(approach.PositionMetres, approach.Rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error), error);
            Reject(s, ownS, "다른 로봇");
            await Until(() => !f.IsTcpMoving, "Peer approach timeout.");
            Reject(s, sharedS, "다른 로봇의 TCP");
            Require(f.TryMoveTcp(returnF.position, returnF.rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error), error);
            await Until(() => !f.IsTcpMoving, "Peer return timeout.");

            Require(c.TryPrepareScaraTransfer(out error) && c.TryExecute(CellRunCommand.Start, out error), error);
            var sequence = s.GetComponent<RobotWaypointSequence>();
            await Until(() => sequence.State == RobotWaypointSequence.RunState.Completed, "Trusted automatic SCARA path timeout.");
            Require(c.TryExecute(CellRunCommand.Stop, out error) && c.TryReleaseControl(out error), error);
            Require(t.HandoffProductId != null && sg.HeldItem == null && f.GetComponent<RobotGripper>().HeldItem == null, "Automatic handoff fixture mismatch.");
            Reject(s, sharedS, "상품 점유"); Reject(f, sharedF, "상품 점유");
            Require(!c.IsFaulted && !t.IsFaulted && t.Count == 1 && t.TryGetItem(t.HandoffProductId, out var original) && original.Owner == null,
                "Admission checks corrupted product/automatic handoff state.");
            var report = new { passed = true, queryReadOnly = true, ownAndEmptySharedAllowedForBoth = true, directApisCannotBypass = true,
                outsidePausedInactiveHeldPeerAndOccupiedRejected = true, actualOwnerPreserved = true, trustedAutomaticScaraHandoffWorks = true,
                newManualInputUi = false, directClicks = 0, callbacksOrUiInputInjected = 0 };
            Directory.CreateDirectory("Logs/LW15"); File.WriteAllText("Logs/LW15/manual-tcp-admission.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;

            void Reject(Component robot, Vector3 goal, string expected)
            {
                string before = State();
                Require(!c.CanMoveManualTcp(robot, goal, out var e) && e.Contains(expected) && State() == before, "Eligibility rejection mismatch: " + e);
                bool accepted = robot == s ? s.TryMoveTcp(goal, s.ActualTcpYawDegrees, ScaraJointLab.IkSelection.Nearest, out _, out e)
                    : f.TryMoveTcp(goal, Quaternion.identity, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out e);
                Require(!accepted && e.Contains(expected) && State() == before, "Direct TCP API bypassed admission: " + e);
            }
            string State() => JsonConvert.SerializeObject(new { c.State, c.FaultReason, records = t.Snapshot(), fifo = t.WaitingSnapshot(),
                slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId }, sv = s.CommandVersion, fv = f.CommandVersion,
                scaraGoal = s.LastAcceptedTcpPosition.HasValue ? s.LastAcceptedTcpPosition.Value.ToString("R") : "none", s.LastAcceptedTcpYawDegrees, fanucGoal = f.LastAcceptedTcpTarget.HasValue ?
                    f.LastAcceptedTcpTarget.Value.position.ToString("R") + f.LastAcceptedTcpTarget.Value.rotation.ToString("R") : "none",
                scara = new[] { s.Axis1.CurrentPosition, s.Axis2.CurrentPosition, s.Axis3.CurrentPosition, s.Axis4.CurrentPosition,
                    s.Axis1.TargetPosition, s.Axis2.TargetPosition, s.Axis3.TargetPosition, s.Axis4.TargetPosition }, fanuc = f.Joints.Select(j => new { j.CurrentPosition, j.TargetPosition }),
                items = l.GetComponentsInChildren<GrippableItem>(true).Select(i => new { i.ProductId, owner = i.Owner != null ? i.Owner.GetInstanceID() : 0 }) });
        }
        finally
        {
            Time.timeScale = scale;
            if (fixture != null) { sg.TryRelease(out _); UnityEngine.Object.Destroy(fixture.gameObject); }
            c.TryExecute(CellRunCommand.Stop, out _);
        }
    }
    private static async Task Until(Func<bool> predicate, string message)
    { var watch = Stopwatch.StartNew(); while (!predicate()) { Require(watch.Elapsed.TotalSeconds < 15, message); await Task.Delay(1); } }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
