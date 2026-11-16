using System;
using UnityEngine;

namespace LineWatch
{
    // Runs after the visual joint controllers, before the physics simulation.
    [DefaultExecutionOrder(100)]
    public sealed class RobotGripper : MonoBehaviour
    {
        public const float PositionToleranceMetres = .002f;
        public const float RotationToleranceDegrees = .5f;
        [SerializeField] private ScaraJointLab scara;
        [SerializeField] private FanucJointLab fanuc;
        [SerializeField] private Transform contact;
        [SerializeField] private Transform leftFinger;
        [SerializeField] private Transform rightFinger;
        [SerializeField] private float fingerTravelMetres;

        private RobotWaypointSequence sequence;
        private Vector3 heldPosition, leftHome, rightHome;
        private Quaternion heldRotation;

        public GrippableItem HeldItem { get; private set; }
        public string LastError { get; private set; }
        public ScaraJointLab Scara => scara;
        public FanucJointLab Fanuc => fanuc;
        public Transform Tcp => scara != null ? scara.Tcp : fanuc != null ? fanuc.Tcp : null;
        public Transform RobotFrame => scara != null ? scara.RobotFrame : fanuc != null ? fanuc.RobotFrame : null;
        public Transform Contact => contact;
        public bool IsReady => isActiveAndEnabled && ((scara != null && fanuc == null && scara.IsReady) ||
            (fanuc != null && scara == null && fanuc.IsReady)) && contact != null && contact.gameObject.activeInHierarchy && Tcp != null && contact.IsChildOf(Tcp) &&
            Vector3.Distance(Tcp.lossyScale, Vector3.one) < .00001f &&
            ((leftFinger == null && rightFinger == null) || (leftFinger != null && rightFinger != null &&
                leftFinger.IsChildOf(Tcp) && rightFinger.IsChildOf(Tcp) && Finite(fingerTravelMetres) && fingerTravelMetres >= 0));
        public bool IsStopped => AxesStopped && (sequence == null || !sequence.IsBusy);
        private bool AxesStopped
        {
            get
            {
                if (!IsReady) return false;
                if (scara != null) return !scara.IsTrajectoryMoving && !scara.Axis1.IsMoving && !scara.Axis2.IsMoving && !scara.Axis3.IsMoving && !scara.Axis4.IsMoving;
                if (fanuc.IsTcpMoving) return false;
                foreach (var joint in fanuc.Joints) if (joint.IsMoving) return false;
                return true;
            }
        }

        private void Awake()
        {
            sequence = GetComponent<RobotWaypointSequence>();
            if (leftFinger != null) leftHome = leftFinger.localPosition;
            if (rightFinger != null) rightHome = rightFinger.localPosition;
        }

        internal bool CanOperate(out string error) => CanOperate(null, false, out error);
        private bool CanOperate(RobotWaypointSequence requester, bool releasing, out string error)
        {
            error = null;
            if (!Application.isPlaying || !IsReady) { error = "Play 상태와 그리퍼·로봇·접촉 기준 연결을 확인하세요."; return false; }
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPaused) { error = "에디터 일시정지를 해제하세요."; return false; }
#endif
            if (Time.timeScale <= 0f) { error = "시간 배율을 복구하세요."; return false; }
            bool sequenceAction = requester != null && requester == sequence && requester.AllowsGripperAction(this, releasing);
            if (!AxesStopped || sequence != null && sequence.IsBusy && !sequenceAction)
            { error = "순차 운전과 모든 축을 정지한 뒤 집기/놓기를 요청하세요."; return false; }
            if (scara != null)
            {
                if (!scara.TryReadFkDiagnostics(out var d, out error)) return false;
                if (d.PositionErrorMetres > .00001f || d.YawErrorDegrees > .005f) error = "SCARA 계산/실제 TCP가 일치하지 않습니다.";
            }
            else
            {
                if (!fanuc.TryReadFkDiagnostics(out var d, out error)) return false;
                if (d.PositionErrorMetres > .00001f || d.RotationErrorDegrees > .005f) error = "FANUC 계산/실제 TCP가 일치하지 않습니다.";
            }
            return error == null;
        }

        public bool TryGrip(GrippableItem item, out string error) => Grip(item, null, out error);
        internal bool TryGripFromSequence(GrippableItem item, RobotWaypointSequence requester, out string error) => Grip(item, requester, out error);
        private bool Grip(GrippableItem item, RobotWaypointSequence requester, out string error)
        {
            if (!CanOperate(requester, false, out error)) return false;
            if (HeldItem != null) { error = "이미 물체를 보유하고 있습니다."; return false; }
            if (item == null || !item.IsReady) { error = "활성 집기 물체·접촉점·키네마틱 본체/콜라이더를 확인하세요."; return false; }
            if (item.Owner != null) { error = "다른 그리퍼가 보유한 물체입니다."; return false; }
            if (Tcp.IsChildOf(item.transform) || item.transform.IsChildOf(transform)) { error = "로봇 계층에 포함된 물체는 집을 수 없습니다."; return false; }
            if (!item.TryGetGripPose(scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc, out var gripPose))
            { error = "이 로봇의 상품 집기 기준이 없습니다."; return false; }
            float positionError = Vector3.Distance(contact.position, gripPose.position);
            float angleError = RotationError(contact.rotation, gripPose.rotation);
            if (!Finite(positionError) || !Finite(angleError) || positionError > PositionToleranceMetres || angleError > RotationToleranceDegrees)
            { error = $"접촉 기준이 맞지 않습니다. 거리 {positionError * 1000:F3}mm / 방향 {angleError:F3}° (허용 2mm / 0.5°)"; return false; }
            var localPosition = Tcp.InverseTransformPoint(item.Body.position);
            var localRotation = Quaternion.Inverse(Tcp.rotation) * item.Body.rotation;
            if (!item.TryClaim(this)) { error = "물체의 보유자를 확보하지 못했습니다."; return false; }
            HeldItem = item; heldPosition = localPosition; heldRotation = localRotation;
            LastError = null; SetFingers(true);
            return true;
        }

        public bool TryRelease(out string error) => Release(null, out error);
        internal bool TryReleaseFromSequence(RobotWaypointSequence requester, out string error) => Release(requester, out error);
        private bool Release(RobotWaypointSequence requester, out string error)
        {
            if (!CanOperate(requester, true, out error)) return false;
            if (HeldItem == null) { error = "보유한 물체가 없습니다."; return false; }
            ClearHeld(); LastError = null;
            return true;
        }

        public void Stop() { if (Application.isPlaying) StopRobot(); }

        internal bool TryCheckPlannedGrip(GrippableItem item, Pose tcpGoal, out string error)
        {
            if (!CanOperate(out error)) return false;
            if (HeldItem != null) { error = "보유 물체를 놓거나 기존 운전을 재개하세요."; return false; }
            if (item == null || !item.IsReady || item.Owner != null) { error = "준비된 미보유 물체가 필요합니다."; return false; }
            if (Tcp.IsChildOf(item.transform) || item.transform.IsChildOf(transform)) { error = "로봇 계층에 포함된 물체입니다."; return false; }
            var worldRotation = RobotFrame.rotation * tcpGoal.rotation;
            var planned = new Pose(RobotFrame.TransformPoint(tcpGoal.position) + worldRotation * Tcp.InverseTransformPoint(contact.position),
                worldRotation * (Quaternion.Inverse(Tcp.rotation) * contact.rotation));
            if (!item.TryGetGripPose(scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc, out var actual))
            { error = "이 로봇의 상품 집기 기준이 없습니다."; return false; }
            float distance = Vector3.Distance(planned.position, actual.position), angle = RotationError(planned.rotation, actual.rotation);
            if (!Finite(distance) || !Finite(angle) || distance > PositionToleranceMetres || angle > RotationToleranceDegrees)
            { error = "물체가 작업점 집기 기준과 맞지 않습니다. 시험 물체를 준비하세요."; return false; }
            return true;
        }

        private void FixedUpdate()
        {
            if (HeldItem == null) return;
            if (!IsReady || !HeldItem.IsReady || HeldItem.Owner != this)
            { StopRobot(); ClearHeld(); LastError = "보유 물체나 그리퍼 설정이 바뀌어 정지하고 보유를 해제했습니다."; return; }
            // Keep the measured relative pose; claiming never snaps the object to a nominal pose.
            HeldItem.Body.MovePosition(Tcp.TransformPoint(heldPosition));
            HeldItem.Body.MoveRotation(Tcp.rotation * heldRotation);
        }

        internal void ItemUnavailable(GrippableItem item)
        {
            if (HeldItem != item) return;
            StopRobot(); ClearHeld(); LastError = "보유 물체가 비활성화/삭제되어 로봇을 정지했습니다.";
        }
        private void ClearHeld()
        {
            if (HeldItem != null) HeldItem.ClearOwner(this);
            HeldItem = null; SetFingers(false);
        }
        private void StopRobot() { if (scara != null) scara.StopAll(); if (fanuc != null) fanuc.StopAll(); }
        private void OnDisable() { if (HeldItem != null) { StopRobot(); ClearHeld(); } }
        private void SetFingers(bool closed)
        {
            if (leftFinger == null || rightFinger == null || Tcp == null) return;
            var displacement = closed ? Tcp.TransformDirection(Vector3.up) * fingerTravelMetres : Vector3.zero;
            leftFinger.localPosition = leftHome - leftFinger.parent.InverseTransformVector(displacement);
            rightFinger.localPosition = rightHome + rightFinger.parent.InverseTransformVector(displacement);
        }
        public static float RotationError(Quaternion a, Quaternion b)
        { var d = Quaternion.Inverse(a) * b; return (float)(2 * Math.Atan2(Math.Sqrt((double)d.x * d.x + (double)d.y * d.y + (double)d.z * d.z), Math.Abs(d.w)) * 180 / Math.PI); }
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
