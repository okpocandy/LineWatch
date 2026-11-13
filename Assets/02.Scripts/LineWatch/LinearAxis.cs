using UnityEngine;

namespace LineWatch
{
    // A kinematic vertical slide. Positive metres move down from the authored home.
    public sealed class LinearAxis : MonoBehaviour
    {
        [SerializeField, Min(0.0001f)] private float speedMetresPerSecond = 0.05f;
        [SerializeField] private float minimumMetres = 0f;
        [SerializeField] private float maximumMetres = 0.15f;

        private Vector3 homePosition;

        public float CurrentPosition { get; private set; }
        public float TargetPosition { get; private set; }
        public bool IsMoving { get; private set; }
        public bool IsAtTarget => Mathf.Abs(CurrentPosition - TargetPosition) < 0.000001f;
        public float MinimumMetres => minimumMetres;
        public float MaximumMetres => maximumMetres;
        public float SpeedMetresPerSecond => speedMetresPerSecond;

        private void Awake()
        {
            homePosition = transform.localPosition;
        }

        public bool MoveTo(float metres)
        {
            if (!isActiveAndEnabled || !Application.isPlaying || float.IsNaN(metres) ||
                float.IsInfinity(metres) || metres < minimumMetres || metres > maximumMetres)
                return false;
            TargetPosition = metres;
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
                speedMetresPerSecond * Time.fixedDeltaTime);
            transform.localPosition = homePosition + Vector3.down * CurrentPosition;
            if (IsAtTarget) IsMoving = false;
        }

        private void OnDisable()
        {
            Stop();
        }
    }
}
