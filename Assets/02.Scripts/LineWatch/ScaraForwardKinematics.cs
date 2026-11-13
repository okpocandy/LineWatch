using System;
using UnityEngine;

namespace LineWatch
{
    public static class ScaraForwardKinematics
    {
        public readonly struct Geometry
        {
            public float Link1Metres { get; }
            public float Link2Metres { get; }
            public float Height12Metres { get; }
            public float Height23Metres { get; }
            // Axis4-to-TCP vector, expressed in RobotFrame axes at zero joint values.
            public Vector3 ToolOffsetMetres { get; }
            public float HomeToolYawDegrees { get; }

            public Geometry(float link1Metres, float link2Metres, float height12Metres,
                float height23Metres, Vector3 toolOffsetMetres, float homeToolYawDegrees = 0f)
            {
                Link1Metres = link1Metres;
                Link2Metres = link2Metres;
                Height12Metres = height12Metres;
                Height23Metres = height23Metres;
                ToolOffsetMetres = toolOffsetMetres;
                HomeToolYawDegrees = homeToolYawDegrees;
            }
        }

        public readonly struct TcpPose
        {
            public Vector3 PositionMetres { get; }
            public float YawDegrees { get; }

            internal TcpPose(Vector3 positionMetres, float yawDegrees)
            {
                PositionMetres = positionMetres;
                YawDegrees = yawDegrees;
            }
        }

        // Inputs are current joint values: rotary degrees and positive-down slide metres.
        // Output is relative to RobotFrame; no Transform, motor command or scene is read.
        public static TcpPose Calculate(float q1Degrees, float q2Degrees, float d3Metres,
            float q4Degrees, Geometry geometry)
        {
            RequireFinite(q1Degrees, nameof(q1Degrees));
            RequireFinite(q2Degrees, nameof(q2Degrees));
            RequireFinite(d3Metres, nameof(d3Metres));
            RequireFinite(q4Degrees, nameof(q4Degrees));
            if (!IsFinite(geometry.Link1Metres) || geometry.Link1Metres <= 0f ||
                !IsFinite(geometry.Link2Metres) || geometry.Link2Metres <= 0f ||
                !IsFinite(geometry.Height12Metres) || !IsFinite(geometry.Height23Metres) ||
                !IsFinite(geometry.ToolOffsetMetres) || !IsFinite(geometry.HomeToolYawDegrees))
                throw new ArgumentException("Geometry must have positive finite link lengths and finite offsets.", nameof(geometry));

            float shoulder = q1Degrees * Mathf.Deg2Rad;
            float elbow = (q1Degrees + q2Degrees) * Mathf.Deg2Rad;
            float totalDegrees = q1Degrees + q2Degrees + q4Degrees;
            float tool = totalDegrees * Mathf.Deg2Rad;
            float sinTool = Mathf.Sin(tool);
            float cosTool = Mathf.Cos(tool);
            Vector3 offset = geometry.ToolOffsetMetres;
            var position = new Vector3(
                geometry.Link1Metres * Mathf.Cos(shoulder) + geometry.Link2Metres * Mathf.Cos(elbow)
                    + offset.x * cosTool + offset.z * sinTool,
                geometry.Height12Metres + geometry.Height23Metres - d3Metres + offset.y,
                -geometry.Link1Metres * Mathf.Sin(shoulder) - geometry.Link2Metres * Mathf.Sin(elbow)
                    - offset.x * sinTool + offset.z * cosTool);
            float yaw = Mathf.DeltaAngle(0f, totalDegrees + geometry.HomeToolYawDegrees);
            if (!IsFinite(position) || !IsFinite(yaw))
                throw new ArgumentException("Joint values or geometry exceed the numeric calculation range.");
            return new TcpPose(position, yaw);
        }

        private static void RequireFinite(float value, string parameter)
        {
            if (!IsFinite(value)) throw new ArgumentOutOfRangeException(parameter, "Value must be finite.");
        }

        private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool IsFinite(Vector3 value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }
}
