using System;
using UnityEngine;
using IK = LineWatch.ScaraInverseKinematics;

namespace LineWatch
{
    public sealed class ScaraJointLab : MonoBehaviour
    {
        public enum IkSelection { Nearest, PositiveElbow, NegativeElbow }
        public enum MotionMode { Joint, Linear }

        [SerializeField] private RotaryAxis axis1;
        [SerializeField] private RotaryAxis axis2;
        [SerializeField] private LinearAxis axis3;
        [SerializeField] private RotaryAxis axis4;
        [SerializeField] private Transform robotFrame;
        [SerializeField] private Transform tcp;
        [SerializeField] private float fkLink1Metres = 0.325f;
        [SerializeField] private float fkLink2Metres = 0.375f;
        [SerializeField] private float fkHeight12Metres = 0.016783f;
        [SerializeField] private float fkHeight23Metres = -0.097783f;
        [SerializeField] private Vector3 fkToolOffsetMetres = new Vector3(0f, -0.034177f, 0f);
        [SerializeField] private float fkHomeToolYawDegrees;

        private ScaraMotionPlanner.Plan activePlan;
        private Vector4 plannedSpeeds;
        private int segmentIndex;
        private float segmentElapsed;

        public bool IsTrajectoryMoving => activePlan != null;
        public string LastMotionError { get; private set; }

        public readonly struct FkDiagnostics
        {
            public ScaraForwardKinematics.TcpPose Calculated { get; }
            public Vector3 ActualPositionMetres { get; }
            public float ActualYawDegrees { get; }
            public float PositionErrorMetres { get; }
            public float YawErrorDegrees { get; }

            internal FkDiagnostics(ScaraForwardKinematics.TcpPose calculated, Vector3 actualPosition, float actualYaw)
            {
                Calculated = calculated;
                ActualPositionMetres = actualPosition;
                ActualYawDegrees = actualYaw;
                PositionErrorMetres = Vector3.Distance(calculated.PositionMetres, actualPosition);
                YawErrorDegrees = Mathf.Abs(Mathf.DeltaAngle(calculated.YawDegrees, actualYaw));
            }
        }

        public RotaryAxis Axis1 => axis1;
        public RotaryAxis Axis2 => axis2;
        public LinearAxis Axis3 => axis3;
        public RotaryAxis Axis4 => axis4;
        public Transform RobotFrame => robotFrame;
        public Transform Tcp => tcp;
        public Vector3 ActualTcpPosition => robotFrame.InverseTransformPoint(tcp.position);
        public float ActualTcpYawDegrees => Vector3.SignedAngle(robotFrame.forward, tcp.forward, robotFrame.up);
        public bool IsReady => isActiveAndEnabled && axis1 != null && axis1.isActiveAndEnabled &&
            axis2 != null && axis2.isActiveAndEnabled && axis3 != null && axis3.isActiveAndEnabled &&
            axis4 != null && axis4.isActiveAndEnabled && robotFrame != null && tcp != null;

        private void Start()
        {
            if (IsReady) return;
            Debug.LogError("[SCARA] Assign all four axes, RobotFrame and TCP.", this);
            enabled = false;
        }

        public bool TryReadFkDiagnostics(out FkDiagnostics diagnostics, out string error)
        {
            diagnostics = default;
            error = null;
            if (!Application.isPlaying || !IsReady)
            {
                error = "Play 상태와 네 축·기준점 연결을 확인하세요.";
                return false;
            }
            try
            {
                var geometry = ReadGeometry();
                var calculated = ScaraForwardKinematics.Calculate(axis1.CurrentPosition, axis2.CurrentPosition,
                    axis3.CurrentPosition, axis4.CurrentPosition, geometry);
                diagnostics = new FkDiagnostics(calculated, ActualTcpPosition, ActualTcpYawDegrees);
                return true;
            }
            catch (ArgumentException exception)
            {
                error = exception.Message;
                return false;
            }
        }

        private ScaraForwardKinematics.Geometry ReadGeometry() => new ScaraForwardKinematics.Geometry(
            fkLink1Metres, fkLink2Metres, fkHeight12Metres, fkHeight23Metres, fkToolOffsetMetres, fkHomeToolYawDegrees);

        public bool TrySelectTcpSolution(Vector3 targetMetres, float yawDegrees, IkSelection selection,
            out IK.Result result, out IK.Solution selected, out string error)
            => TrySelectTcpSolution(targetMetres, yawDegrees, selection, MotionMode.Joint, out result, out selected, out error);

        public bool TrySelectTcpSolution(Vector3 targetMetres, float yawDegrees, IkSelection selection, MotionMode motion,
            out IK.Result result, out IK.Solution selected, out string error)
            => TryPrepareMotion(targetMetres, yawDegrees, selection, motion, out result, out selected, out _, out error);

        private bool TryPrepareMotion(Vector3 targetMetres, float yawDegrees, IkSelection selection, MotionMode motion,
            out IK.Result result, out IK.Solution selected, out ScaraMotionPlanner.Plan plan, out string error)
        {
            result = null;
            selected = default;
            plan = null;
            error = null;
            if (!Application.isPlaying || !IsReady)
            {
                error = "Play 상태와 네 축·기준점 연결을 확인하세요.";
                return false;
            }
            if (selection != IkSelection.Nearest && selection != IkSelection.PositiveElbow && selection != IkSelection.NegativeElbow)
            {
                error = "유효한 해 선택 방식을 지정하세요.";
                return false;
            }
            if (motion != MotionMode.Joint && motion != MotionMode.Linear)
            { error = "유효한 이동 방식을 지정하세요."; return false; }
            var limits = new IK.JointLimits(
                new IK.JointValues(axis1.MinimumDegrees, axis2.MinimumDegrees, axis3.MinimumMetres, axis4.MinimumDegrees),
                new IK.JointValues(axis1.MaximumDegrees, axis2.MaximumDegrees, axis3.MaximumMetres, axis4.MaximumDegrees));
            var geometry = ReadGeometry();
            result = IK.Solve(targetMetres, yawDegrees, geometry, limits);
            if (!result.Success)
            {
                switch (result.Failure)
                {
                    case IK.FailureReason.Unreachable: error = "팔이 닿지 않는 목표입니다."; break;
                    case IK.FailureReason.JointLimit: error = "관절 허용 범위 안에서 도달할 수 없습니다."; break;
                    case IK.FailureReason.Indeterminate: error = "이 목표는 관절 해를 하나로 정할 수 없습니다."; break;
                    case IK.FailureReason.NumericalError: error = "계산 오차가 허용값을 넘었습니다."; break;
                    default: error = "목표·기하 입력·관절 범위가 유효한 숫자인지 확인하세요."; break;
                }
                return false;
            }
            double bestCost = double.PositiveInfinity;
            bool found = false;
            foreach (var candidate in result.Solutions)
            {
                if (selection == IkSelection.PositiveElbow && candidate.ElbowSign < 0 ||
                    selection == IkSelection.NegativeElbow && candidate.ElbowSign > 0) continue;
                var j = candidate.Joints;
                double d1 = (double)j.Q1Degrees - axis1.CurrentPosition;
                double d2 = (double)j.Q2Degrees - axis2.CurrentPosition;
                double d4 = (double)j.Q4Degrees - axis4.CurrentPosition;
                double cost = d1 * d1 + d2 * d2 + d4 * d4;
                if (cost >= bestCost) continue;
                bestCost = cost;
                selected = candidate;
                found = true;
            }
            if (!found)
            {
                error = "선택한 굽힘 방향에 유효한 해가 없습니다. 해 선택을 바꿔보세요.";
                return false;
            }
            var start = CurrentJoints();
            var speeds = CurrentSpeeds();
            ScaraMotionPlanner.Failure failure;
            bool prepared;
            if (motion == MotionMode.Linear)
            {
                if (!TryReadFkDiagnostics(out var diagnostics, out error)) return false;
                if (diagnostics.PositionErrorMetres > 0.00001f || diagnostics.YawErrorDegrees > 0.005f)
                { error = "기하 입력과 실제 TCP가 일치하지 않아 직선 경로를 검증할 수 없습니다."; return false; }
                int branch = selection == IkSelection.PositiveElbow ? 1 : selection == IkSelection.NegativeElbow ? -1 : 0;
                prepared = ScaraMotionPlanner.Linear(start, targetMetres, yawDegrees, geometry, limits, speeds, branch, out plan, out failure);
            }
            else prepared = ScaraMotionPlanner.Joint(start, selected, limits, speeds, out plan, out failure);
            if (!prepared)
            {
                error = failure == ScaraMotionPlanner.Failure.ConfigurationChangeRequired
                    ? "현재 굽힘·회전 범위로 직선 경로를 이을 수 없습니다. 관절 이동 또는 경유점을 사용하세요."
                    : failure == ScaraMotionPlanner.Failure.ResolutionLimit ? "경로 분할 한도 안에서 오차를 확인할 수 없습니다. 경유점을 나눠주세요."
                    : failure == ScaraMotionPlanner.Failure.PathUnreachable ? "목표 또는 중간 경로가 도달 범위·관절 제한을 벗어납니다."
                    : "기하·관절 범위·축 속도를 확인하세요.";
                return false;
            }
            selected = plan.End;
            return true;
        }

        public bool TryMoveTcp(Vector3 targetMetres, float yawDegrees, IkSelection selection,
            out IK.Solution selected, out string error)
            => TryMoveTcp(targetMetres, yawDegrees, selection, MotionMode.Joint, out selected, out error);

        public bool TryMoveTcp(Vector3 targetMetres, float yawDegrees, IkSelection selection, MotionMode motion,
            out IK.Solution selected, out string error)
        {
            selected = default;
            error = null;
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPaused)
            {
                error = "일시정지를 해제한 후 이동하세요.";
                return false;
            }
#endif
            if (Time.timeScale <= 0f)
            {
                error = "일시정지를 해제한 후 이동하세요.";
                return false;
            }
            // Recalculate on every request: a displayed candidate may have stale geometry or limits.
            if (!TryPrepareMotion(targetMetres, yawDegrees, selection, motion, out _, out selected, out var plan, out error)) return false;
            var j = selected.Joints;
            if (!axis1.CanMoveTo(j.Q1Degrees) || !axis2.CanMoveTo(j.Q2Degrees) ||
                !axis3.CanMoveTo(j.D3Metres) || !axis4.CanMoveTo(j.Q4Degrees))
            {
                selected = default;
                error = "네 축의 활성 상태와 허용 범위를 확인하세요.";
                return false;
            }
            // Commit only the complete validated plan; a failed replacement preserves the old one.
            activePlan = plan;
            plannedSpeeds = CurrentSpeeds();
            segmentIndex = 1;
            segmentElapsed = 0f;
            LastMotionError = null;
            axis1.BeginCoordinated(j.Q1Degrees);
            axis2.BeginCoordinated(j.Q2Degrees);
            axis3.BeginCoordinated(j.D3Metres);
            axis4.BeginCoordinated(j.Q4Degrees);
            return true;
        }

        private IK.JointValues CurrentJoints() => new IK.JointValues(axis1.CurrentPosition, axis2.CurrentPosition, axis3.CurrentPosition, axis4.CurrentPosition);
        private Vector4 CurrentSpeeds() => new Vector4(axis1.SpeedDegreesPerSecond, axis2.SpeedDegreesPerSecond, axis3.SpeedMetresPerSecond, axis4.SpeedDegreesPerSecond);

        private void FixedUpdate()
        {
            if (activePlan == null) return;
            var end = activePlan.End.Joints;
            if (!IsReady || !axis1.IsCoordinated || !axis2.IsCoordinated || !axis3.IsCoordinated || !axis4.IsCoordinated ||
                !CurrentSpeeds().Equals(plannedSpeeds) || !CanApply(end))
            { StopAll(); LastMotionError = "경로 이동 중 축 상태·설정이 바뀌어 정지했습니다."; return; }
            float remaining = Time.fixedDeltaTime;
            while (activePlan != null)
            {
                var before = activePlan.Nodes[segmentIndex - 1];
                var after = activePlan.Nodes[segmentIndex];
                float duration = after.SecondsFromPrevious;
                float step = Mathf.Min(remaining, Mathf.Max(0f, duration - segmentElapsed));
                segmentElapsed += step;
                remaining -= step;
                bool segmentDone = duration <= 0f || segmentElapsed >= duration;
                var position = ScaraMotionPlanner.Lerp(before.Joints, after.Joints, segmentDone ? 1f : segmentElapsed / duration);
                if (!CanApply(position))
                { StopAll(); LastMotionError = "경로의 관절값이 현재 허용 범위를 벗어나 정지했습니다."; return; }
                bool complete = segmentDone && segmentIndex == activePlan.Nodes.Count - 1;
                axis1.ApplyCoordinated(position.Q1Degrees, complete);
                axis2.ApplyCoordinated(position.Q2Degrees, complete);
                axis3.ApplyCoordinated(position.D3Metres, complete);
                axis4.ApplyCoordinated(position.Q4Degrees, complete);
                if (complete) { activePlan = null; break; }
                if (segmentDone) { segmentIndex++; segmentElapsed = 0f; }
                if (remaining <= 0f) break;
            }
        }

        private bool CanApply(IK.JointValues j) => axis1.CanMoveTo(j.Q1Degrees) && axis2.CanMoveTo(j.Q2Degrees) &&
            axis3.CanMoveTo(j.D3Metres) && axis4.CanMoveTo(j.Q4Degrees);

        public bool MoveJoint(int index, float target)
        {
            if (!Application.isPlaying || !IsReady) return false;
            bool allowed;
            switch (index)
            {
                case 0: allowed = axis1.CanMoveTo(target); break;
                case 1: allowed = axis2.CanMoveTo(target); break;
                case 2: allowed = axis3.CanMoveTo(target); break;
                case 3: allowed = axis4.CanMoveTo(target); break;
                default: return false;
            }
            if (!allowed) return false;
            if (activePlan != null) StopAll();
            LastMotionError = null;
            bool accepted;
            switch (index)
            {
                case 0: accepted = axis1.MoveTo(target); break;
                case 1: accepted = axis2.MoveTo(target); break;
                case 2: accepted = axis3.MoveTo(target); break;
                case 3: accepted = axis4.MoveTo(target); break;
                default: return false;
            }
            return accepted;
        }

        public void StopJoint(int index)
        {
            if (!Application.isPlaying || !IsReady) return;
            if (index >= 0 && index <= 3 && activePlan != null) { StopAll(); return; }
            switch (index)
            {
                case 0: axis1.Stop(); break;
                case 1: axis2.Stop(); break;
                case 2: axis3.Stop(); break;
                case 3: axis4.Stop(); break;
            }
        }

        public void HomeAll()
        {
            if (!Application.isPlaying || !IsReady) return;
            if (activePlan != null) StopAll();
            LastMotionError = null;
            axis1.MoveTo(0f);
            axis2.MoveTo(0f);
            axis3.MoveTo(0f);
            axis4.MoveTo(0f);
        }

        public void StopAll()
        {
            activePlan = null;
            if (!Application.isPlaying) return;
            if (axis1 != null) axis1.Stop();
            if (axis2 != null) axis2.Stop();
            if (axis3 != null) axis3.Stop();
            if (axis4 != null) axis4.Stop();
        }

        private void OnDisable()
        {
            StopAll();
        }
    }
}
