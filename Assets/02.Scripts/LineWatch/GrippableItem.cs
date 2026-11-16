using UnityEngine;

namespace LineWatch
{
    [RequireComponent(typeof(Rigidbody), typeof(BoxCollider))]
    public sealed class GrippableItem : MonoBehaviour
    {
        [SerializeField] private Transform gripPoint;
        private Rigidbody body;
        private BoxCollider shape;

        public RobotGripper Owner { get; private set; }
        public Transform GripPoint => gripPoint;
        public Rigidbody Body => body;
        // Transform can lag a Rigidbody pose assignment until the next physics sync.
        public Pose GripPose => new Pose(body.position + body.rotation * transform.InverseTransformPoint(gripPoint.position),
            body.rotation * (Quaternion.Inverse(transform.rotation) * gripPoint.rotation));
        public bool IsReady => isActiveAndEnabled && body != null && body.isKinematic && !body.useGravity && body.detectCollisions &&
            shape != null && shape.enabled && !shape.isTrigger && gripPoint != null && gripPoint.gameObject.activeInHierarchy && gripPoint.IsChildOf(transform) &&
            Vector3.Distance(transform.lossyScale, Vector3.one) < .00001f;

        private void Awake() { body = GetComponent<Rigidbody>(); shape = GetComponent<BoxCollider>(); }

        internal bool TryClaim(RobotGripper gripper)
        {
            if (!IsReady || Owner != null || gripper == null) return false;
            Owner = gripper;
            return true;
        }

        internal void ClearOwner(RobotGripper gripper) { if (Owner == gripper) Owner = null; }

        private void OnDisable()
        {
            if (Owner != null) Owner.ItemUnavailable(this);
        }
    }
}
