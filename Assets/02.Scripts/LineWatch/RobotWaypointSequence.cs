using System;
using System.Collections.Generic;
using UnityEngine;
using SIK = LineWatch.ScaraInverseKinematics;
using FIK = LineWatch.FanucInverseKinematics;

namespace LineWatch
{
    public sealed class RobotWaypointSequence : MonoBehaviour
    {
        public enum RunState { Idle, Running, Paused, Faulted, Completed, Acting }
        [SerializeField] private RobotWaypointSet waypoints;
        [SerializeField] private ScaraJointLab scara;
        [SerializeField] private FanucJointLab fanuc;

        private RobotWaypointSet source;
        private CellCommandGate cellGate;
        private void Awake() { cellGate = new CellCommandGate(this); }
        private ScaraJointLab scaraOwner;
        private FanucJointLab fanucOwner;
        private readonly Pose[] targets = new Pose[4];
        private uint ownedVersion;
        private ScaraForwardKinematics.Geometry scaraGeometry;
        private RobotGripper transportGripper;
        private GrippableItem transportItem;

        public RobotWaypointSet Waypoints => waypoints;
        public RunState State { get; private set; }
        public int StepIndex { get; private set; }
        public string LastError { get; private set; }
        public string LastAcknowledgedFaultReason { get; private set; }
        public int LastArrivedStepIndex { get; private set; } = -1;
        public Pose LastArrivalPose { get; private set; }
        public bool IsBusy => State == RunState.Running || State == RunState.Acting;
        public bool IsPickPlace { get; private set; }
        public GrippableItem CycleItem => transportItem;
        public int GripCount { get; private set; }
        public int ReleaseCount { get; private set; }
        public Pose LastReleasePose { get; private set; }
        public string StepName => StepIndex == 0 ? "접근" : StepIndex == 1 ? "작업" : StepIndex == 2 ? "상승" : StepIndex == 3 ? "퇴피" : "완료";
        public bool IsReady => isActiveAndEnabled && waypoints != null &&
            ((scara != null && fanuc == null && scara.IsReady && waypoints.Robot == RobotWaypointSet.RobotType.Scara) ||
             (fanuc != null && scara == null && fanuc.IsReady && waypoints.Robot == RobotWaypointSet.RobotType.Fanuc));

        public bool TryStart(out string error) => StartProgram(null, null, out error);
        public bool TryStartPickPlace(RobotGripper gripper, GrippableItem item, out string error)
        {
            if (gripper == null || item == null) { error = "그리퍼와 준비된 시험 물체를 확인하세요."; return false; }
            return StartProgram(gripper, item, out error);
        }
        private bool StartProgram(RobotGripper gripper, GrippableItem item, out string error)
        {
            if (!CanRequest(out error)) return false;
            if (IsBusy) { error = "실행 중입니다. 정지 후 다시 시작하세요."; return false; }
            if (IsPickPlace && transportGripper != null && transportGripper.HeldItem != null)
            { error = "보유 물체를 놓거나 기존 운전을 재개하세요."; return false; }
            if (!AtRest(scara, fanuc)) { error = "수동 이동이 끝나거나 정지한 뒤 시작하세요."; return false; }
            if (!waypoints.TryValidate(out error)) return false;
            var candidate = ReadTargets(waypoints);
            if (gripper != null)
            {
                if (gripper.gameObject != gameObject || gripper.Scara != scara || gripper.Fanuc != fanuc)
                { error = "이 순차 운전과 같은 로봇의 그리퍼가 필요합니다."; return false; }
                if (!gripper.TryCheckPlannedGrip(item, candidate[1], out error)) return false;
            }
            if (!Preflight(scara, fanuc, candidate, 0, out error)) return false;
            if (!Dispatch(scara, fanuc, candidate[0], 0, out error)) return false;
            source = waypoints; scaraOwner = scara; fanucOwner = fanuc;
            Array.Copy(candidate, targets, 4);
            if (scaraOwner != null) scaraGeometry = scaraOwner.Geometry;
            ownedVersion = Version;
            StepIndex = 0; LastError = null; State = RunState.Running;
            LastArrivedStepIndex = -1;
            IsPickPlace = gripper != null; transportGripper = gripper; transportItem = item;
            GripCount = 0; ReleaseCount = 0;
            return true;
        }

        public bool TryResume(out string error)
        {
            if (!CanRequest(out error)) return false;
            return Resume(out error);
        }

        internal bool TryResumeFromCell(CellRunController owner, out string error)
        {
            if (owner == null || owner.ControlledSequence != this || !cellGate.IsManagedBy(owner))
            { error = "이 경로를 인계받은 셀 관리자만 재개할 수 있습니다."; return false; }
            if (!cellGate.CanIssue(this, out error)) return false;
            return Resume(out error);
        }

        // Read-only preparation. Cell Start/Resume checks this before changing its state;
        // execution validates again against the latest joints/program before dispatch.
        internal bool TryValidateResume(out string error)
        {
            if (!CanReadRequest(out error)) return false;
            if (State != RunState.Paused && State != RunState.Faulted) { error = "중단된 순서가 없습니다."; return false; }
            if (!SameProgram()) { error = "경유점이나 로봇 연결이 바뀌었습니다. 처음부터 시작하세요."; return false; }
            if (!AtRest(scaraOwner, fanucOwner)) { error = "현재 수동 이동을 정지한 뒤 재개하세요."; return false; }
            if (!CheckTransport(out error)) return false;
            if (IsPickPlace && GripCount == 0 && !transportGripper.TryCheckPlannedGrip(transportItem, targets[1], out error)) return false;
            if (!Preflight(scaraOwner, fanucOwner, targets, StepIndex, out error)) return false;
            return true;
        }

        // Cell recovery preserves the saved step and action counts. A pick/place program
        // can be acknowledged before its grip or while the same item is still held.
        internal bool CanAcknowledgeFault(CellRunController owner, out string error)
        {
            if (!CanReadRequest(out error)) return false;
            if (State != RunState.Faulted || owner == null || !owner.isActiveAndEnabled || !owner.IsFaulted ||
                !cellGate.IsManagedBy(owner) || owner.ControlledSequence != this)
            { error = "셀이 인계받은 고장 경로만 공통 해제할 수 있습니다."; return false; }
            if (IsPickPlace)
            {
                bool beforeGrip = GripCount == 0 && ReleaseCount == 0 && StepIndex >= 0 && StepIndex <= 1;
                bool holding = GripCount == 1 && ReleaseCount == 0 && StepIndex >= 1 && StepIndex <= 3;
                if (!beforeGrip && !holding)
                { error = "저장 단계와 집기/놓기 횟수가 복구 범위와 맞지 않습니다. 놓기 완료 후 복구는 별도 지원이 필요합니다."; return false; }
                if (!CheckTransport(out error)) return false;
            }
            if (!SameProgram()) { error = "저장된 경유점과 로봇 연결을 원상 복원하세요."; return false; }
            return true;
        }

        // Only the coordinator calls this after all paths/FK and the final tracker check
        // succeed. No callbacks, dispatch or action counters change during the commits.
        internal void AcknowledgeFaultFromCell()
        {
            LastAcknowledgedFaultReason = LastError;
            LastError = null;
            State = RunState.Paused;
        }

        private bool Resume(out string error)
        {
            if (!TryValidateResume(out error)) return false;
            if (!Dispatch(scaraOwner, fanucOwner, targets[StepIndex], StepIndex, out error)) return false;
            if (scaraOwner != null) scaraGeometry = scaraOwner.Geometry;
            ownedVersion = Version; LastError = null; State = RunState.Running;
            return true;
        }

        public void Pause()
        {
            if (!Application.isPlaying || !IsBusy) return;
            StopOwned(); State = RunState.Paused; LastError = null;
        }

        private void Update()
        {
            if (State != RunState.Running) return;
            if (!SameProgram() || !IsReady) { Fault("경유점·로봇 연결·활성 상태가 바뀌었습니다."); return; }
            if (!CheckTransport(out var transportError)) { Fault(transportError); return; }
            if (Version != ownedVersion)
            {
                if (!string.IsNullOrEmpty(MotionError)) { Fault(MotionError); return; }
                State = RunState.Paused;
                LastError = "수동/외부 명령으로 중단했습니다. 재개 전 현재 자세를 확인하세요.";
                return; // The newer command owns the robot; do not stop or overwrite it.
            }
            if (scaraOwner != null && !scaraOwner.Geometry.Equals(scaraGeometry)) { Fault("SCARA 기하 입력이 바뀌었습니다."); return; }
            if (!string.IsNullOrEmpty(MotionError)) { Fault(MotionError); return; }
            if (!ModelAligned(out var modelError)) { Fault(modelError); return; }
            if (!ClockRunning() || Moving || !AtRest(scaraOwner, fanucOwner)) return;
            if (!AtTarget(out var actual, out var error)) { Fault(error); return; }
            LastArrivedStepIndex = StepIndex; LastArrivalPose = actual;
            if (IsPickPlace && !PerformAction(out error)) { Fault(error); return; }
            if (State != RunState.Running) return;
            if (StepIndex == 3) { StepIndex = 4; State = RunState.Completed; return; }
            int next = StepIndex + 1;
            // Advance the logical step only after a fresh move request succeeds.
            if (!Dispatch(scaraOwner, fanucOwner, targets[next], next, out error)) { Fault(error); return; }
            StepIndex = next; ownedVersion = Version;
        }

        private bool CanRequest(out string error)
        {
            if (!cellGate.CanIssue(null, out error)) return false;
            return CanReadRequest(out error);
        }

        private bool CanReadRequest(out string error)
        {
            error = null;
            if (!Application.isPlaying || !IsReady) { error = "Play 상태와 경유점·로봇 연결을 확인하세요."; return false; }
            if (!ClockRunning()) { error = "일시정지와 시간 배율을 해제한 뒤 요청하세요."; return false; }
            return true;
        }

        private static bool ClockRunning()
        {
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPaused) return false;
#endif
            return Time.timeScale > 0f;
        }

        private bool SameProgram()
        {
            if (source == null || source != waypoints || scara != scaraOwner || fanuc != fanucOwner || !source.TryValidate(out _)) return false;
            return Same(targets[0], source.Approach) && Same(targets[1], source.Work) && Same(targets[3], source.Retreat);
        }
        private static bool Same(Pose pose, RobotWaypointSet.Target target) => pose.position.Equals(target.PositionMetres) && pose.rotation.Equals(target.Rotation);
        private static Pose[] ReadTargets(RobotWaypointSet set) => new[] { new Pose(set.Approach.PositionMetres, set.Approach.Rotation),
            new Pose(set.Work.PositionMetres, set.Work.Rotation), new Pose(set.Approach.PositionMetres, set.Approach.Rotation), new Pose(set.Retreat.PositionMetres, set.Retreat.Rotation) };
        private uint Version => scaraOwner != null ? scaraOwner.CommandVersion : fanucOwner != null ? fanucOwner.CommandVersion : 0;
        private bool Moving => scaraOwner != null ? scaraOwner.IsTrajectoryMoving : fanucOwner != null && fanucOwner.IsTcpMoving;
        private string MotionError => scaraOwner != null ? scaraOwner.LastMotionError : fanucOwner != null ? fanucOwner.LastMotionError : null;

        private static bool AtRest(ScaraJointLab s, FanucJointLab f)
        {
            if (s != null) return !s.IsTrajectoryMoving && !s.Axis1.IsMoving && !s.Axis2.IsMoving && !s.Axis3.IsMoving && !s.Axis4.IsMoving;
            if (f == null || f.IsTcpMoving) return false;
            foreach (var joint in f.Joints) if (joint == null || joint.IsMoving) return false;
            return true;
        }

        private void StopOwned()
        {
            if (Version != ownedVersion) return;
            if (scaraOwner != null) scaraOwner.StopAll();
            if (fanucOwner != null) fanucOwner.StopAll();
        }
        private void Fault(string error) { StopOwned(); State = RunState.Faulted; LastError = error; }
        private void OnDisable() { if (IsBusy) { StopOwned(); State = RunState.Paused; } }

        private bool CheckTransport(out string error)
        {
            error = null;
            if (!IsPickPlace) return true;
            if (transportGripper == null || !transportGripper.IsReady || transportGripper.Scara != scaraOwner || transportGripper.Fanuc != fanucOwner ||
                transportItem == null || !transportItem.IsReady)
                error = "운반 물체나 그리퍼가 유효하지 않습니다. 상태를 확인하고 새 운전을 시작하세요.";
            else if (GripCount == 0 && (transportGripper.HeldItem != null || transportItem.Owner != null))
                error = "집기 전 물체/그리퍼의 보유자가 바뀌었습니다.";
            else if (GripCount > ReleaseCount && (transportGripper.HeldItem != transportItem || transportItem.Owner != transportGripper))
                error = "운반 물체의 보유가 해제되거나 바뀌었습니다. 자동 재집기하지 않습니다.";
            return error == null;
        }

        internal bool AllowsGripperAction(RobotGripper gripper, bool releasing) => IsPickPlace && State == RunState.Acting &&
            gripper == transportGripper && Version == ownedVersion &&
            (releasing ? StepIndex == 3 && GripCount == 1 && ReleaseCount == 0 : StepIndex == 1 && GripCount == 0);

        private bool PerformAction(out string error)
        {
            error = null;
            if (StepIndex == 1 && GripCount == 0)
            {
                State = RunState.Acting;
                if (!transportGripper.TryGripFromSequence(transportItem, this, out error)) return false;
                GripCount++;
                if (State == RunState.Acting) State = RunState.Running;
            }
            if (StepIndex == 3 && ReleaseCount == 0)
            {
                State = RunState.Acting;
                if (!transportGripper.TryReleaseFromSequence(this, out error)) return false;
                LastReleasePose = new Pose(transportItem.Body.position, transportItem.Body.rotation);
                ReleaseCount++;
                if (State == RunState.Acting) State = RunState.Running;
            }
            return true;
        }

        private bool AtTarget(out Pose actual, out string error)
        {
            var target = targets[StepIndex]; error = null; actual = default;
            if (scaraOwner != null)
            {
                if (!scaraOwner.TryReadFkDiagnostics(out var d, out error)) return false;
                actual = new Pose(d.ActualPositionMetres, Quaternion.Euler(0, d.ActualYawDegrees, 0));
                if (d.PositionErrorMetres > .00001f || d.YawErrorDegrees > .005f ||
                    Vector3.Distance(d.ActualPositionMetres, target.position) > .002f || Mathf.Abs(Mathf.DeltaAngle(d.ActualYawDegrees, Yaw(target.rotation))) > .5f)
                    error = "SCARA 실제 TCP가 목표/계산과 일치하지 않습니다.";
            }
            else
            {
                if (!fanucOwner.TryReadFkDiagnostics(out var d, out error)) return false;
                actual = new Pose(d.ActualPositionMetres, d.ActualRotation);
                if (d.PositionErrorMetres > .00001f || d.RotationErrorDegrees > .005f ||
                    Vector3.Distance(d.ActualPositionMetres, target.position) > .002f || Angle(d.ActualRotation, target.rotation) > .5f)
                    error = "FANUC 실제 TCP가 목표/계산과 일치하지 않습니다.";
            }
            return error == null;
        }

        private bool Dispatch(ScaraJointLab s, FanucJointLab f, Pose target, int index, out string error)
        {
            if (s != null) return s.TryMoveTcpFromSequence(target.position, Yaw(target.rotation), ScaraJointLab.IkSelection.PositiveElbow,
                index == 0 ? ScaraJointLab.MotionMode.Joint : ScaraJointLab.MotionMode.Linear, this, out _, out error);
            return f.TryMoveTcpFromSequence(target.position, target.rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, this, out _, out error);
        }

        private bool ModelAligned(out string error)
        {
            error = null;
            if (scaraOwner != null)
            {
                if (!scaraOwner.TryReadFkDiagnostics(out var d, out error)) return false;
                if (d.PositionErrorMetres > .00001f || d.YawErrorDegrees > .005f) error = "SCARA 이동 중 계산/실측이 다릅니다.";
            }
            else
            {
                if (!fanucOwner.TryReadFkDiagnostics(out var d, out error)) return false;
                if (d.PositionErrorMetres > .00001f || d.RotationErrorDegrees > .005f) error = "FANUC 이동 중 계산/실측이 다릅니다.";
            }
            return error == null;
        }

        private static bool Preflight(ScaraJointLab s, FanucJointLab f, Pose[] program, int first, out string error)
        {
            error = null;
            if (s != null)
            {
                if (!s.TryReadFkDiagnostics(out var d, out error)) return false;
                if (d.PositionErrorMetres > .00001f || d.YawErrorDegrees > .005f) { error = "SCARA 모델/기하 기준이 일치하지 않습니다."; return false; }
                var limits = new SIK.JointLimits(J(s.Axis1.MinimumDegrees, s.Axis2.MinimumDegrees, s.Axis3.MinimumMetres, s.Axis4.MinimumDegrees),
                    J(s.Axis1.MaximumDegrees, s.Axis2.MaximumDegrees, s.Axis3.MaximumMetres, s.Axis4.MaximumDegrees));
                var speeds = new Vector4(s.Axis1.SpeedDegreesPerSecond, s.Axis2.SpeedDegreesPerSecond, s.Axis3.SpeedMetresPerSecond, s.Axis4.SpeedDegreesPerSecond);
                var current = J(s.Axis1.CurrentPosition, s.Axis2.CurrentPosition, s.Axis3.CurrentPosition, s.Axis4.CurrentPosition);
                for (int i = first; i < 4; i++)
                {
                    var goal = program[i]; float yaw = Yaw(goal.rotation);
                    var result = SIK.Solve(goal.position, yaw, s.Geometry, limits);
                    bool found = false; double best = double.PositiveInfinity; SIK.Solution end = default;
                    foreach (var candidate in result.Solutions)
                    {
                        if (candidate.ElbowSign < 0) continue;
                        var q = candidate.Joints; double cost = Math.Pow(q.Q1Degrees - current.Q1Degrees, 2) +
                            Math.Pow(q.Q2Degrees - current.Q2Degrees, 2) + Math.Pow(q.Q4Degrees - current.Q4Degrees, 2);
                        if (cost < best) { found = true; best = cost; end = candidate; }
                    }
                    if (!result.Success || !found) { error = $"{i + 1}/4 경유점 IK 사전 검사 실패: {result.Failure}"; return false; }
                    bool ok = i == 0 ? ScaraMotionPlanner.Joint(current, end, limits, speeds, out var plan, out var failure)
                        : ScaraMotionPlanner.Linear(current, goal.position, yaw, s.Geometry, limits, speeds, 1, out plan, out failure);
                    if (!ok) { error = $"{i + 1}/4 경유점 경로 사전 검사 실패: {failure}"; return false; }
                    current = plan.End.Joints;
                }
            }
            else
            {
                if (!f.TryPreviewTcp(program[first].position, program[first].rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out var preview, out error)) return false;
                var current = new float[6]; var lower = new float[6]; var upper = new float[6];
                for (int i = 0; i < 6; i++)
                {
                    var axis = f.Joints[i];
                    if (float.IsNaN(axis.SpeedDegreesPerSecond) || float.IsInfinity(axis.SpeedDegreesPerSecond) || axis.SpeedDegreesPerSecond <= 0f)
                    { error = "FANUC의 양의 유한 속도를 확인하세요."; return false; }
                    current[i] = axis.CurrentPosition; lower[i] = axis.MinimumDegrees; upper[i] = axis.MaximumDegrees;
                }
                for (int i = first; i < 4; i++)
                {
                    var seeds = new List<float[]> { current, new float[6], new[] { 10f, -15f, 25f, 15f, 20f, -10f }, new[] { -10f, 15f, -25f, -15f, -20f, 10f } };
                    var result = i == first ? preview.Result : FIK.Solve(program[i].position, program[i].rotation, f.Geometry, lower, upper, seeds);
                    if (!result.Succeeded) { error = $"{i + 1}/4 경유점 IK 사전 검사 실패: {result.Reason}"; return false; }
                    current = new float[6]; for (int j = 0; j < 6; j++) current[j] = result.JointDegrees[j];
                }
            }
            return true;
        }
        private static SIK.JointValues J(float a, float b, float c, float d) => new SIK.JointValues(a, b, c, d);
        private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
        private static float Angle(Quaternion a, Quaternion b)
        { var d = Quaternion.Inverse(a) * b; return (float)(2 * Math.Atan2(Math.Sqrt((double)d.x * d.x + (double)d.y * d.y + (double)d.z * d.z), Math.Abs(d.w)) * 180 / Math.PI); }
    }
}
