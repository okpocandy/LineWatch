using UnityEngine;

namespace LineWatch
{
    // A kinematic visual joint. Angles are degrees relative to its authored home pose.
    public sealed class RotaryAxis : MonoBehaviour
    {
        [SerializeField, Min(0.01f)] private float speedDegreesPerSecond = 30f;
        [SerializeField] private float minimumDegrees = -170f;
        [SerializeField] private float maximumDegrees = 170f;
        [SerializeField] private Vector3 localRotationAxis = Vector3.up;

        private Quaternion homeRotation;
        internal bool IsCoordinated { get; private set; }

        public float CurrentPosition { get; private set; }
        public float TargetPosition { get; private set; }
        public bool IsMoving { get; private set; }
        public bool IsAtTarget => Mathf.Abs(CurrentPosition - TargetPosition) < 0.001f;
        public float SpeedDegreesPerSecond => speedDegreesPerSecond;
        public float MinimumDegrees => minimumDegrees;
        public float MaximumDegrees => maximumDegrees;
        public Vector3 LocalRotationAxis => localRotationAxis.normalized;
        public Quaternion HomeRotation => homeRotation;

        private void Awake()
        {
            homeRotation = transform.localRotation;
        }

        public bool CanMoveTo(float degrees) => isActiveAndEnabled && Application.isPlaying &&
            !float.IsNaN(degrees) && !float.IsInfinity(degrees) && degrees >= minimumDegrees && degrees <= maximumDegrees;

        public bool MoveTo(float degrees)
        {
            if (!CanMoveTo(degrees)) return false;
            IsCoordinated = false;
            TargetPosition = degrees;
            IsMoving = !IsAtTarget;
            return true;
        }

        public void Stop()
        {
            IsCoordinated = false;
            IsMoving = false;
        }

        internal void BeginCoordinated(float target)
        {
            TargetPosition = target;
            IsCoordinated = true;
            IsMoving = CurrentPosition != target;
        }

        internal void ApplyCoordinated(float position, bool complete)
        {
            CurrentPosition = position;
            transform.localRotation = homeRotation * Quaternion.AngleAxis(position, LocalRotationAxis);
            if (complete) { IsMoving = false; IsCoordinated = false; }
        }

        private void FixedUpdate()
        {
            if (!IsMoving || IsCoordinated) return;

            CurrentPosition = Mathf.MoveTowards(CurrentPosition, TargetPosition,
                speedDegreesPerSecond * Time.fixedDeltaTime);
            transform.localRotation = homeRotation * Quaternion.AngleAxis(CurrentPosition, LocalRotationAxis);
            if (IsAtTarget) IsMoving = false;
        }

        private void OnDisable()
        {
            Stop();
        }
    }
}
