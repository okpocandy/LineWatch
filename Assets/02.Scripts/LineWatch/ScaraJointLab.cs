using System;
using UnityEngine;
using IK = LineWatch.ScaraInverseKinematics;

namespace LineWatch
{
    public sealed class ScaraJointLab : MonoBehaviour
    {
        public enum IkSelection { Nearest, PositiveElbow, NegativeElbow }

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
        {
            result = null;
            selected = default;
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
            var limits = new IK.JointLimits(
                new IK.JointValues(axis1.MinimumDegrees, axis2.MinimumDegrees, axis3.MinimumMetres, axis4.MinimumDegrees),
                new IK.JointValues(axis1.MaximumDegrees, axis2.MaximumDegrees, axis3.MaximumMetres, axis4.MaximumDegrees));
            result = IK.Solve(targetMetres, yawDegrees, ReadGeometry(), limits);
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
            if (!found) error = "선택한 굽힘 방향에 유효한 해가 없습니다. 해 선택을 바꿔보세요.";
            return found;
        }

        public bool TryMoveTcp(Vector3 targetMetres, float yawDegrees, IkSelection selection,
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
            if (!TrySelectTcpSolution(targetMetres, yawDegrees, selection, out _, out selected, out error)) return false;
            var j = selected.Joints;
            if (!axis1.CanMoveTo(j.Q1Degrees) || !axis2.CanMoveTo(j.Q2Degrees) ||
                !axis3.CanMoveTo(j.D3Metres) || !axis4.CanMoveTo(j.Q4Degrees))
            {
                selected = default;
                error = "네 축의 활성 상태와 허용 범위를 확인하세요.";
                return false;
            }
            // All four checks precede any command; these synchronous setters have no callbacks.
            axis1.MoveTo(j.Q1Degrees);
            axis2.MoveTo(j.Q2Degrees);
            axis3.MoveTo(j.D3Metres);
            axis4.MoveTo(j.Q4Degrees);
            return true;
        }

        public bool MoveJoint(int index, float target)
        {
            if (!Application.isPlaying || !IsReady) return false;
            switch (index)
            {
                case 0: return axis1.MoveTo(target);
                case 1: return axis2.MoveTo(target);
                case 2: return axis3.MoveTo(target);
                case 3: return axis4.MoveTo(target);
                default: return false;
            }
        }

        public void StopJoint(int index)
        {
            if (!Application.isPlaying || !IsReady) return;
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
            axis1.MoveTo(0f);
            axis2.MoveTo(0f);
            axis3.MoveTo(0f);
            axis4.MoveTo(0f);
        }

        public void StopAll()
        {
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
