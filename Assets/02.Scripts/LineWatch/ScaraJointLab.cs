using System;
using UnityEngine;

namespace LineWatch
{
    public sealed class ScaraJointLab : MonoBehaviour
    {
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
                var geometry = new ScaraForwardKinematics.Geometry(fkLink1Metres, fkLink2Metres,
                    fkHeight12Metres, fkHeight23Metres, fkToolOffsetMetres, fkHomeToolYawDegrees);
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
