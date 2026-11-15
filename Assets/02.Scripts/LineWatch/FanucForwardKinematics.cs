using System;
using System.Collections.Generic;
using UnityEngine;

namespace LineWatch
{
    public static class FanucForwardKinematics
    {
        public readonly struct TcpPose
        {
            public Vector3 PositionMetres { get; }
            public Quaternion Rotation { get; }

            internal TcpPose(Vector3 positionMetres, Quaternion rotation)
            {
                PositionMetres = positionMetres;
                Rotation = rotation;
            }
        }

        // RobotFrame-relative pose only: no Transform reads, commands or motor-limit clamping.
        public static TcpPose Calculate(IReadOnlyList<float> jointDegrees, FanucKinematicGeometry geometry)
        {
            if (jointDegrees == null || jointDegrees.Count != 6)
                throw new ArgumentException("Exactly six joint angles are required.", nameof(jointDegrees));
            if (geometry == null || geometry.Joints == null || geometry.Joints.Count != 6)
                throw new ArgumentException("Exactly six joint home transforms are required.", nameof(geometry));

            var position = Vector3.zero;
            var rotation = Quaternion.identity;
            for (int i = 0; i < 6; i++)
            {
                if (!Finite(jointDegrees[i]))
                    throw new ArgumentOutOfRangeException(nameof(jointDegrees), "Joint angles must be finite.");
                var home = geometry.Joints[i];
                if (home == null || !Finite(home.PositionMetres) || !Unit(home.Rotation) ||
                    !Finite(home.LocalRotationAxis) || !Finite(home.LocalRotationAxis.sqrMagnitude) ||
                    home.LocalRotationAxis.sqrMagnitude < 1e-12f)
                    throw new ArgumentException("Homes need finite positions, unit rotations and nonzero finite axes.", nameof(geometry));

                position += rotation * home.PositionMetres;
                rotation = Quaternion.Normalize(rotation * home.Rotation *
                    Quaternion.AngleAxis(jointDegrees[i], home.LocalRotationAxis.normalized));
            }
            if (!Finite(geometry.ToolPositionMetres) || !Unit(geometry.ToolRotation))
                throw new ArgumentException("Tool pose needs a finite position and unit rotation.", nameof(geometry));
            position += rotation * geometry.ToolPositionMetres;
            rotation = Quaternion.Normalize(rotation * geometry.ToolRotation);
            if (!Finite(position) || !Unit(rotation))
                throw new ArgumentException("Joint values or geometry exceed the numeric calculation range.");
            return new TcpPose(position, rotation);
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool Finite(Vector3 value) => Finite(value.x) && Finite(value.y) && Finite(value.z);
        private static bool Unit(Quaternion value)
        {
            if (!Finite(value.x) || !Finite(value.y) || !Finite(value.z) || !Finite(value.w)) return false;
            double norm = (double)value.x * value.x + (double)value.y * value.y +
                (double)value.z * value.z + (double)value.w * value.w;
            return Math.Abs(norm - 1.0) <= 0.0001;
        }
    }
}
