using System;
using System.Collections.Generic;
using UnityEngine;

namespace LineWatch
{
    // Authored home transforms in metres, independent of the robot's world placement.
    // Joint motion will be composed as T(home position) * R(home) * R(axis, q).
    [Serializable]
    public sealed class FanucKinematicGeometry
    {
        [Serializable]
        public sealed class JointHome
        {
            [SerializeField] private Vector3 positionMetres;
            [SerializeField] private Quaternion rotation = Quaternion.identity;
            [SerializeField] private Vector3 localRotationAxis;

            public Vector3 PositionMetres => positionMetres;
            public Quaternion Rotation => rotation;
            public Vector3 LocalRotationAxis => localRotationAxis;

            public JointHome(Vector3 positionMetres, Quaternion rotation, Vector3 localRotationAxis)
            {
                this.positionMetres = positionMetres;
                this.rotation = rotation;
                this.localRotationAxis = localRotationAxis;
            }
        }

        [SerializeField] private JointHome[] joints = new JointHome[6];
        [SerializeField] private Vector3 toolPositionMetres;
        [SerializeField] private Quaternion toolRotation = Quaternion.identity;

        public IReadOnlyList<JointHome> Joints => joints;
        // Fixed A6 -> provided TCP transform, including its orientation.
        public Vector3 ToolPositionMetres => toolPositionMetres;
        public Quaternion ToolRotation => toolRotation;

        public FanucKinematicGeometry() { }

        public FanucKinematicGeometry(JointHome[] joints, Vector3 toolPositionMetres, Quaternion toolRotation)
        {
            this.joints = joints == null ? null : (JointHome[])joints.Clone();
            this.toolPositionMetres = toolPositionMetres;
            this.toolRotation = toolRotation;
        }
    }
}
