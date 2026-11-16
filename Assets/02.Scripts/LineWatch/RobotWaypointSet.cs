using System;
using UnityEngine;

namespace LineWatch
{
    // Authored targets only; execution progress belongs to a robot instance.
    [CreateAssetMenu(fileName = "RobotWaypoints", menuName = "LineWatch/Robot Waypoints")]
    public sealed class RobotWaypointSet : ScriptableObject
    {
        public enum RobotType { Scara, Fanuc }

        [Serializable]
        public sealed class Target
        {
            [Tooltip("TCP position in metres, relative to this robot's RobotFrame.")]
            [SerializeField] private Vector3 positionMetres;
            [Tooltip("TCP orientation relative to RobotFrame. SCARA uses a Y-axis rotation only.")]
            [SerializeField] private Quaternion rotation = Quaternion.identity;

            public Vector3 PositionMetres => positionMetres;
            public Quaternion Rotation => rotation;
        }

        [SerializeField] private RobotType robot;
        [SerializeField] private Target approach = new Target();
        [SerializeField] private Target work = new Target();
        [SerializeField] private Target retreat = new Target();

        public RobotType Robot => robot;
        public Target Approach => approach;
        public Target Work => work;
        public Target Retreat => retreat;
        // Planned order: approach -> work -> approach (lift) -> retreat.

        public bool TryValidate(out string error)
        {
            error = null;
            if (robot != RobotType.Scara && robot != RobotType.Fanuc)
                error = "로봇 종류를 확인하세요.";
            else if (!Valid(approach) || !Valid(work) || !Valid(retreat))
                error = "경유점의 유한한 위치(m)와 단위 Quaternion을 확인하세요.";
            else if (Vector2.Distance(new Vector2(approach.PositionMetres.x, approach.PositionMetres.z),
                new Vector2(work.PositionMetres.x, work.PositionMetres.z)) > .000001f ||
                approach.PositionMetres.y <= work.PositionMetres.y || retreat.PositionMetres.y < approach.PositionMetres.y - .000001f ||
                Vector3.Distance(approach.PositionMetres, retreat.PositionMetres) < .0001f)
                error = "접근점은 작업점 바로 위, 퇴피점은 접근 높이 이상인 별도 위치로 정하세요.";
            else if (RotationDifference(approach.Rotation, work.Rotation) > .01f || RotationDifference(retreat.Rotation, work.Rotation) > .01f)
                error = "이번 고정 경유점에서는 TCP 방향을 동일하게 유지하세요.";
            else if (robot == RobotType.Scara && Vector3.Distance(work.Rotation * Vector3.up, Vector3.up) > .00001f)
                error = "SCARA는 RobotFrame의 Y축 회전만 지정할 수 있습니다.";
            return error == null;
        }

        private static bool Valid(Target target)
        {
            if (target == null) return false;
            var p = target.PositionMetres; var q = target.Rotation;
            return Finite(p.x) && Finite(p.y) && Finite(p.z) && Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w) &&
                Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) < .0001f;
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

        private static float RotationDifference(Quaternion a, Quaternion b)
        {
            var d = Quaternion.Inverse(a) * b;
            return (float)(2 * Math.Atan2(Math.Sqrt((double)d.x * d.x + (double)d.y * d.y + (double)d.z * d.z), Math.Abs(d.w)) * 180 / Math.PI);
        }
    }
}
