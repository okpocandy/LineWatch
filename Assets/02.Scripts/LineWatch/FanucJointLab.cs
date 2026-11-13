using System.Collections.Generic;
using UnityEngine;

namespace LineWatch
{
    public sealed class FanucJointLab : MonoBehaviour
    {
        [SerializeField] private RotaryAxis[] joints = new RotaryAxis[6];
        [SerializeField] private Transform tcp;

        public IReadOnlyList<RotaryAxis> Joints => joints;
        public Transform Tcp => tcp;
        public bool IsReady
        {
            get
            {
                if (!isActiveAndEnabled || joints.Length != 6 || tcp == null) return false;
                foreach (var joint in joints)
                    if (joint == null || !joint.isActiveAndEnabled) return false;
                return true;
            }
        }

        private void Start()
        {
            if (IsReady) return;
            Debug.LogError("[FANUC] Assign all six rotary axes and the TCP.", this);
            enabled = false;
        }

        public bool MoveJoint(int index, float degrees)
        {
            return Application.isPlaying && IsReady && index >= 0 && index < joints.Length && joints[index].MoveTo(degrees);
        }

        public void HomeAll()
        {
            if (!Application.isPlaying || !IsReady) return;
            foreach (var joint in joints) joint.MoveTo(0f);
        }

        public void StopAll()
        {
            if (!Application.isPlaying) return;
            foreach (var joint in joints)
                if (joint != null) joint.Stop();
        }

        private void OnDisable()
        {
            StopAll();
        }
    }
}
