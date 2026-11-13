using UnityEngine;

namespace LineWatch
{
    // A kinematic visual joint. Angles are degrees relative to its authored home pose.
    public sealed class RotaryAxis : MonoBehaviour
    {
        [SerializeField, Min(0.01f)] private float speedDegreesPerSecond = 30f;
        [SerializeField] private float minimumDegrees = -170f;
        [SerializeField] private float maximumDegrees = 170f;

        private Quaternion homeRotation;

        public float CurrentPosition { get; private set; }
        public float TargetPosition { get; private set; }
        public bool IsMoving { get; private set; }
        public bool IsAtTarget => Mathf.Abs(CurrentPosition - TargetPosition) < 0.001f;
        public float SpeedDegreesPerSecond => speedDegreesPerSecond;

        private void Awake()
        {
            homeRotation = transform.localRotation;
        }

        public bool MoveTo(float degrees)
        {
            if (!isActiveAndEnabled || !Application.isPlaying || float.IsNaN(degrees) ||
                float.IsInfinity(degrees) || degrees < minimumDegrees || degrees > maximumDegrees)
                return false;

            TargetPosition = degrees;
            IsMoving = !IsAtTarget;
            return true;
        }

        public void Stop()
        {
            IsMoving = false;
        }

        private void FixedUpdate()
        {
            if (!IsMoving) return;

            CurrentPosition = Mathf.MoveTowards(CurrentPosition, TargetPosition,
                speedDegreesPerSecond * Time.fixedDeltaTime);
            transform.localRotation = homeRotation * Quaternion.AngleAxis(CurrentPosition, Vector3.up);
            if (IsAtTarget) IsMoving = false;
        }

        private void OnDisable()
        {
            Stop();
        }
    }
}
