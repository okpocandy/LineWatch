using UnityEngine;

namespace LineWatch
{
    [RequireComponent(typeof(Rigidbody), typeof(BoxCollider))]
    public sealed class GrippableItem : MonoBehaviour
    {
        [SerializeField] private Transform gripPoint;
        [SerializeField] private Transform scaraGripPoint;
        [SerializeField] private Transform fanucGripPoint;
        private Rigidbody body;
        private BoxCollider shape;

        public RobotGripper Owner { get; private set; }
        public Transform GripPoint => gripPoint;
        public Rigidbody Body => body;
        // Transform can lag a Rigidbody pose assignment until the next physics sync.
        // Default frame for legacy diagnostics; robot commands use TryGetGripPose.
        public Pose GripPose => WorldGripPose(gripPoint);
        public bool IsReady => isActiveAndEnabled && body != null && body.isKinematic && !body.useGravity && body.detectCollisions &&
            shape != null && shape.enabled && !shape.isTrigger && ValidPoint(gripPoint) &&
            ((scaraGripPoint == null && fanucGripPoint == null) || (ValidPoint(scaraGripPoint) && ValidPoint(fanucGripPoint))) &&
            Vector3.Distance(transform.lossyScale, Vector3.one) < .00001f;

        // Existing single-point parts keep their original behavior. A dual-point part needs both frames.
        public Transform GetGripPoint(RobotWaypointSet.RobotType robot)
        {
            if (robot != RobotWaypointSet.RobotType.Scara && robot != RobotWaypointSet.RobotType.Fanuc) return null;
            if (scaraGripPoint == null && fanucGripPoint == null) return gripPoint;
            return robot == RobotWaypointSet.RobotType.Scara ? scaraGripPoint : fanucGripPoint;
        }

        public bool TryGetGripPose(RobotWaypointSet.RobotType robot, out Pose pose)
        {
            pose = default;
            var point = GetGripPoint(robot);
            if (!IsReady || !ValidPoint(point)) return false;
            pose = WorldGripPose(point);
            return true;
        }

        private bool ValidPoint(Transform point) => point != null && point.gameObject.activeInHierarchy && point.IsChildOf(transform);
        private Pose WorldGripPose(Transform point) => new Pose(body.position + body.rotation * transform.InverseTransformPoint(point.position),
            body.rotation * (Quaternion.Inverse(transform.rotation) * point.rotation));

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
