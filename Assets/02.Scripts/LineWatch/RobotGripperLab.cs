using UnityEngine;

namespace LineWatch
{
    public sealed class RobotGripperLab : MonoBehaviour
    {
        [SerializeField] private RobotGripper gripper;
        [SerializeField] private RobotWaypointSet waypoints;
        [SerializeField] private GrippableItem trialPrefab;
        private RobotWaypointSequence sequence;

        private void Awake() { sequence = GetComponent<RobotWaypointSequence>(); }

        public bool TryStartPickPlace(out string error)
        {
            error = null;
            if (!Application.isPlaying || !IsReady || sequence == null) { error = "Play 상태와 자동 집기 운전 연결을 확인하세요."; return false; }
            return sequence.TryStartPickPlace(gripper, TrialItem, out error);
        }

        public RobotGripper Gripper => gripper;
        public GrippableItem TrialItem { get; private set; }
        public bool IsReady => isActiveAndEnabled && gripper != null && gripper.IsReady && waypoints != null && trialPrefab != null &&
            ((gripper.Scara != null && waypoints.Robot == RobotWaypointSet.RobotType.Scara) ||
             (gripper.Fanuc != null && waypoints.Robot == RobotWaypointSet.RobotType.Fanuc));

        public bool TryPrepare(out string error)
        {
            error = null;
            if (!IsReady) { error = "집기 시험의 로봇·경유점·시험 물체 연결을 확인하세요."; return false; }
            if (!gripper.CanOperate(out error)) return false;
            if (gripper.HeldItem != null || TrialItem != null && TrialItem.Owner != null)
            { error = "보유 중인 시험 물체를 초기화할 수 없습니다."; return false; }
            if (!waypoints.TryValidate(out error)) return false;
            if (TrialItem != null && !TrialItem.IsReady) { TrialItem.gameObject.SetActive(false); Destroy(TrialItem.gameObject); TrialItem = null; }
            if (TrialItem == null) TrialItem = Instantiate(trialPrefab);
            TrialItem.name = (gripper.Scara != null ? "SCARA" : "FANUC") + "_GripTrial";
            var tcp = gripper.Tcp; var frame = gripper.RobotFrame;
            var worldTcpRotation = frame.rotation * waypoints.Work.Rotation;
            var worldTcpPosition = frame.TransformPoint(waypoints.Work.PositionMetres);
            var point = worldTcpPosition + worldTcpRotation * tcp.InverseTransformPoint(gripper.Contact.position);
            var rotation = worldTcpRotation * (Quaternion.Inverse(tcp.rotation) * gripper.Contact.rotation);
            var item = TrialItem;
            var offset = item.transform.InverseTransformPoint(item.GripPoint.position);
            var orientation = Quaternion.Inverse(item.transform.rotation) * item.GripPoint.rotation;
            var bodyRotation = rotation * Quaternion.Inverse(orientation);
            // Explicit test reset of an unowned kinematic body, before any motion command.
            item.Body.position = point - bodyRotation * offset; item.Body.rotation = bodyRotation;
            return true;
        }

        public bool TryMoveToWork(out string error) => Move(false, out error);
        public bool TryLift(out string error) => Move(true, out error);
        private bool Move(bool lift, out string error)
        {
            error = null;
            if (!IsReady) { error = "집기 시험 연결을 확인하세요."; return false; }
            if (!gripper.CanOperate(out error) || !waypoints.TryValidate(out error)) return false;
            var target = lift ? waypoints.Approach : waypoints.Work;
            if (gripper.Scara != null) return gripper.Scara.TryMoveTcp(target.PositionMetres,
                Vector3.SignedAngle(Vector3.forward, target.Rotation * Vector3.forward, Vector3.up), ScaraJointLab.IkSelection.PositiveElbow,
                lift ? ScaraJointLab.MotionMode.Linear : ScaraJointLab.MotionMode.Joint, out _, out error);
            return gripper.Fanuc.TryMoveTcp(target.PositionMetres, target.Rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error);
        }
    }
}
