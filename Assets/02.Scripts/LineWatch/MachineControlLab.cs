using UnityEngine;

namespace LineWatch
{
    public sealed class MachineControlLab : MonoBehaviour
    {
        [SerializeField] private RotaryAxis axis;
        [SerializeField] private PresenceSensor sensor;
        [SerializeField] private Rigidbody probe;
        [SerializeField] private float testAngleDegrees = 30f;

        private bool probeMovePending;
        private Vector3 probeTarget;

        public RotaryAxis Axis => axis;
        public PresenceSensor TestSensor => sensor;
        public Rigidbody Probe => probe;
        public float TestAngleDegrees => testAngleDegrees;
        public bool IsReady => isActiveAndEnabled && axis != null && sensor != null && probe != null;

        private void Start()
        {
            if (IsReady) return;
            Debug.LogError("[LW-03] Axis, Sensor and Probe must be assigned.", this);
            enabled = false;
        }

        [ContextMenu("LW-03/축을 시험 각도로 이동")]
        public void MoveToTestAngle()
        {
            if (CanOperate()) axis.MoveTo(testAngleDegrees);
        }

        [ContextMenu("LW-03/축을 0도로 복귀")]
        public void MoveHome()
        {
            if (CanOperate()) axis.MoveTo(0f);
        }

        [ContextMenu("LW-03/축 정지")]
        public void StopAxis()
        {
            if (CanOperate()) axis.Stop();
        }

        [ContextMenu("LW-03/시험 물체를 센서 안으로")]
        public void MoveProbeInside()
        {
            if (CanOperate()) QueueProbeMove(sensor.transform.position);
        }

        [ContextMenu("LW-03/시험 물체를 센서 밖으로")]
        public void MoveProbeOutside()
        {
            if (CanOperate()) QueueProbeMove(sensor.transform.position + sensor.transform.right * 0.6f);
        }

        private void QueueProbeMove(Vector3 position)
        {
            probeTarget = position;
            probeMovePending = true;
        }

        private void FixedUpdate()
        {
            if (!probeMovePending) return;
            probe.MovePosition(probeTarget);
            probeMovePending = false;
        }

        private bool CanOperate()
        {
            if (Application.isPlaying && IsReady) return true;
            Debug.LogWarning("[LW-03] Open RobotLab and enter Play Mode before operating the lab.", this);
            return false;
        }

        private void OnDisable()
        {
            probeMovePending = false;
            if (Application.isPlaying && axis != null) axis.Stop();
        }
    }
}
