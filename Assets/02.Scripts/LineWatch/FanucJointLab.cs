using System;
using System.Collections.Generic;
using UnityEngine;

namespace LineWatch
{
    public sealed class FanucJointLab : MonoBehaviour
    {
        public enum IkSeedPolicy { CurrentThenFixed, CurrentOnly, Fixed }
        [SerializeField] private RotaryAxis[] joints = new RotaryAxis[6];
        [SerializeField] private Transform tcp;
        [SerializeField] private Transform robotFrame;
        [SerializeField] private FanucKinematicGeometry geometry = new FanucKinematicGeometry();

        private sealed class TcpMotion
        {
            public readonly RotaryAxis[] Axes = new RotaryAxis[6];
            public readonly float[] Start = new float[6], End = new float[6], Speeds = new float[6];
            public readonly float[] Lower = new float[6], Upper = new float[6], Applied = new float[6], Next = new float[6];
            public FanucKinematicGeometry Geometry;
            public Transform Frame, Tcp;
            public double Duration, Elapsed;
        }

        private TcpMotion activeMotion;
        public bool IsTcpMoving => activeMotion != null;
        public string LastMotionError { get; private set; }

        public IReadOnlyList<RotaryAxis> Joints => joints;
        public Transform Tcp => tcp;
        public Transform RobotFrame => robotFrame;
        public FanucKinematicGeometry Geometry => geometry;

        public readonly struct IkPreview
        {
            public FanucInverseKinematics.Result Result { get; }
            public string SeedName { get; }
            public int AvailableSeeds { get; }
            public IReadOnlyList<float> CurrentJointDegrees { get; }

            internal IkPreview(FanucInverseKinematics.Result result, string seedName, int availableSeeds, float[] current)
            {
                Result = result;
                SeedName = seedName;
                AvailableSeeds = availableSeeds;
                CurrentJointDegrees = Array.AsReadOnly(current);
            }
        }

        public readonly struct FkDiagnostics
        {
            public FanucForwardKinematics.TcpPose Calculated { get; }
            public Vector3 ActualPositionMetres { get; }
            public Quaternion ActualRotation { get; }
            public float PositionErrorMetres { get; }
            public float RotationErrorDegrees { get; }

            internal FkDiagnostics(FanucForwardKinematics.TcpPose calculated, Vector3 actualPosition, Quaternion actualRotation)
            {
                Calculated = calculated;
                ActualPositionMetres = actualPosition;
                ActualRotation = actualRotation;
                PositionErrorMetres = Vector3.Distance(calculated.PositionMetres, actualPosition);
                RotationErrorDegrees = RotationDifferenceDegrees(calculated.Rotation, actualRotation);
            }
        }
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

        public bool TryReadActualTcp(out Vector3 positionMetres, out Quaternion rotation, out string error)
        {
            positionMetres = default;
            rotation = Quaternion.identity;
            error = null;
            if (!Application.isPlaying || robotFrame == null || tcp == null)
            {
                error = "Play 상태와 RobotFrame·TCP 연결을 확인하세요.";
                return false;
            }
            positionMetres = robotFrame.InverseTransformPoint(tcp.position);
            rotation = Quaternion.Inverse(robotFrame.rotation) * tcp.rotation;
            if (!Finite(positionMetres.x) || !Finite(positionMetres.y) || !Finite(positionMetres.z) ||
                !Finite(rotation.x) || !Finite(rotation.y) || !Finite(rotation.z) || !Finite(rotation.w))
            {
                error = "실제 TCP 변환에 유한하지 않은 값이 있습니다.";
                return false;
            }
            return true;
        }

        public bool TryReadFkDiagnostics(out FkDiagnostics diagnostics, out string error)
        {
            diagnostics = default;
            if (!TryReadActualTcp(out var actualPosition, out var actualRotation, out error)) return false;
            if (!IsReady)
            {
                error = "여섯 축의 연결·활성 상태를 확인하세요.";
                return false;
            }
            try
            {
                var current = new float[6];
                for (int i = 0; i < current.Length; i++) current[i] = joints[i].CurrentPosition;
                var calculated = FanucForwardKinematics.Calculate(current, geometry);
                diagnostics = new FkDiagnostics(calculated, actualPosition, actualRotation);
                return true;
            }
            catch (ArgumentException exception)
            {
                error = "FK 기하 입력을 확인하세요.\n" + exception.Message;
                return false;
            }
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

        public bool TryPreviewTcp(Vector3 targetMetres, Quaternion targetRotation, IkSeedPolicy policy,
            out IkPreview preview, out string error)
        {
            preview = default;
            if (!Enum.IsDefined(typeof(IkSeedPolicy), policy))
            {
                error = "초기값 정책을 확인하세요.";
                return false;
            }
            if (!TryReadFkDiagnostics(out var diagnostics, out error)) return false;
            if (diagnostics.PositionErrorMetres > .00001f || diagnostics.RotationErrorDegrees > .005f)
            {
                error = "저장 기하와 실제 모델이 일치하지 않습니다. FK 진단·RobotFrame·TCP 연결을 확인하세요.";
                return false;
            }
            if (!MatchesModel(geometry))
            {
                error = "6축 계층·홈 변환·회전축·단위 스케일과 저장 기하의 연결을 확인하세요.";
                return false;
            }
            var current = new float[6]; var lower = new float[6]; var upper = new float[6];
            for (int i = 0; i < 6; i++)
            {
                current[i] = joints[i].CurrentPosition;
                lower[i] = joints[i].MinimumDegrees;
                upper[i] = joints[i].MaximumDegrees;
            }
            var seeds = new List<float[]>(); var names = new List<string>();
            void Add(float[] values, string name)
            {
                foreach (var existing in seeds)
                {
                    bool same = true;
                    for (int i = 0; i < 6; i++) same &= Mathf.Abs(existing[i] - values[i]) <= .0001f;
                    if (same) return;
                }
                seeds.Add(values);
                names.Add(name);
            }
            if (policy != IkSeedPolicy.Fixed) Add(current, "현재 자세");
            if (policy != IkSeedPolicy.CurrentOnly)
            {
                Add(new float[6], "홈");
                Add(new[] { 10f, -15f, 25f, 15f, 20f, -10f }, "고정 대체 1");
                Add(new[] { -10f, 15f, -25f, -15f, -20f, 10f }, "고정 대체 2");
            }
            var result = FanucInverseKinematics.Solve(targetMetres, targetRotation, geometry, lower, upper, seeds);
            string selected = result.SeedIndex >= 0 ? names[result.SeedIndex] : "계산 전 종료";
            preview = new IkPreview(result, selected, seeds.Count, current);
            error = PreviewFailure(result.Reason);
            return result.Succeeded;
        }

        private static string PreviewFailure(FanucInverseKinematics.Failure reason)
        {
            switch (reason)
            {
                case FanucInverseKinematics.Failure.None: return null;
                case FanucInverseKinematics.Failure.InvalidInput: return "목표 위치·단위 Quaternion 등 입력을 확인하세요. 0 Quaternion은 사용할 수 없습니다.";
                case FanucInverseKinematics.Failure.InvalidGeometry: return "FK 홈 기하 입력이 유효하지 않습니다.";
                case FanucInverseKinematics.Failure.InvalidSeed: return "초기 관절값이 제한 밖이거나 유효하지 않습니다. 초기값 정책·관절 제한을 확인하세요.";
                case FanucInverseKinematics.Failure.OutsideReachBound: return "목표가 도달 범위의 보수적 외곽 상한 밖입니다.";
                case FanucInverseKinematics.Failure.IterationLimit: return "반복 예산 안에서 해를 찾지 못했습니다. 초기값 정책이나 목표를 바꿔 확인하세요.";
                case FanucInverseKinematics.Failure.TimeLimit: return "시간 예산 안에서 해를 찾지 못했습니다. 초기값 정책이나 목표를 바꿔 확인하세요.";
                case FanucInverseKinematics.Failure.JointLimitStall: return "관절 경계에서 탐색이 정체했습니다. 도달 불가의 증명은 아니며 초기값 정책을 바꿔 확인하세요.";
                case FanucInverseKinematics.Failure.Stalled: return "탐색이 정체해 해를 찾지 못했습니다. 도달 불가의 증명은 아닙니다.";
                default: return "수치 계산을 완료하지 못했습니다. 목표·기하 입력을 확인하세요.";
            }
        }

        public bool TryMoveTcp(Vector3 targetMetres, Quaternion targetRotation, IkSeedPolicy policy,
            out IkPreview preview, out string error)
        {
            preview = default;
            error = null;
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPaused)
            {
                error = "일시정지를 해제한 후 이동하세요.";
                return false;
            }
#endif
            if (Time.timeScale <= 0f)
            {
                error = "시간 배율을 복구한 후 이동하세요.";
                return false;
            }
            // Always solve from the latest state; displayed preview joints are never executed.
            if (!TryPreviewTcp(targetMetres, targetRotation, policy, out preview, out error)) return false;
            var candidate = new TcpMotion { Frame = robotFrame, Tcp = tcp };
            var homes = new FanucKinematicGeometry.JointHome[6];
            for (int i = 0; i < 6; i++)
            {
                var axis = joints[i];
                float start = axis.CurrentPosition, end = preview.Result.JointDegrees[i], speed = axis.SpeedDegreesPerSecond;
                if (!axis.CanMoveTo(start) || !axis.CanMoveTo(end) || !Finite(speed) || speed <= 0f)
                {
                    error = "여섯 축의 현재값·목표·허용 범위·양의 속도를 확인하세요.";
                    return false;
                }
                candidate.Axes[i] = axis;
                candidate.Start[i] = candidate.Applied[i] = start;
                candidate.End[i] = end;
                candidate.Speeds[i] = speed;
                candidate.Lower[i] = axis.MinimumDegrees;
                candidate.Upper[i] = axis.MaximumDegrees;
                candidate.Duration = Math.Max(candidate.Duration, Math.Abs((double)end - start) / speed);
                var home = geometry.Joints[i];
                homes[i] = new FanucKinematicGeometry.JointHome(home.PositionMetres, home.Rotation, home.LocalRotationAxis);
            }
            candidate.Geometry = new FanucKinematicGeometry(homes, geometry.ToolPositionMetres, geometry.ToolRotation);
            candidate.Duration *= 1.00001;
            // Commit only after every axis and the whole straight joint-space segment are valid.
            activeMotion = candidate;
            LastMotionError = null;
            for (int i = 0; i < 6; i++) candidate.Axes[i].BeginCoordinated(candidate.End[i]);
            if (candidate.Duration == 0)
            {
                for (int i = 0; i < 6; i++) candidate.Axes[i].ApplyCoordinated(candidate.End[i], true);
                activeMotion = null;
            }
            return true;
        }

        private void FixedUpdate()
        {
            var plan = activeMotion;
            if (plan == null) return;
            bool valid = IsReady && robotFrame == plan.Frame && tcp == plan.Tcp &&
                SameGeometry(geometry, plan.Geometry) && MatchesModel(plan.Geometry);
            for (int i = 0; i < 6 && valid; i++)
            {
                var axis = joints[i];
                valid = axis == plan.Axes[i] && axis.IsCoordinated && axis.CurrentPosition == plan.Applied[i] &&
                    axis.TargetPosition == plan.End[i] && axis.SpeedDegreesPerSecond == plan.Speeds[i] &&
                    axis.MinimumDegrees == plan.Lower[i] && axis.MaximumDegrees == plan.Upper[i];
            }
            if (!valid || !Finite(Time.fixedDeltaTime) || Time.fixedDeltaTime <= 0f)
            { AbortMotion("관절 계획 이동 중 축·기하·연결·설정이 바뀌어 전체 정지했습니다."); return; }
            double elapsed = Math.Min(plan.Duration, plan.Elapsed + Time.fixedDeltaTime);
            float fraction = (float)(elapsed / plan.Duration);
            bool complete = elapsed >= plan.Duration;
            for (int i = 0; i < 6; i++)
            {
                plan.Next[i] = complete ? plan.End[i] : Mathf.Lerp(plan.Start[i], plan.End[i], fraction);
                if (!plan.Axes[i].CanMoveTo(plan.Next[i]))
                { AbortMotion("계획 관절값이 현재 허용 범위를 벗어나 전체 정지했습니다."); return; }
            }
            plan.Elapsed = elapsed;
            for (int i = 0; i < 6; i++)
            {
                plan.Axes[i].ApplyCoordinated(plan.Next[i], complete);
                plan.Applied[i] = plan.Next[i];
            }
            if (complete) activeMotion = null;
        }

        private void AbortMotion(string error) { StopAll(); LastMotionError = error; }

        private bool MatchesModel(FanucKinematicGeometry expected)
        {
            if (!IsReady || robotFrame == null || expected?.Joints == null || expected.Joints.Count != 6 ||
                !Near(robotFrame.lossyScale, Vector3.one, .000001f)) return false;
            Transform parent = robotFrame;
            for (int i = 0; i < 6; i++)
            {
                var axis = joints[i]; var home = expected.Joints[i]; var t = axis.transform;
                if (home == null || t.parent != parent || !Near(t.localScale, Vector3.one, .000001f) ||
                    !Near(t.localPosition, home.PositionMetres, .00001f) ||
                    !Near(axis.LocalRotationAxis, home.LocalRotationAxis.normalized, .000001f) ||
                    !NearRotation(axis.HomeRotation, home.Rotation) ||
                    !NearRotation(t.localRotation, home.Rotation * Quaternion.AngleAxis(axis.CurrentPosition, axis.LocalRotationAxis)))
                    return false;
                parent = t;
            }
            return tcp != null && tcp.parent == parent && Vector3.Distance(tcp.localScale, Vector3.one) <= .000001f &&
                Vector3.Distance(tcp.localPosition, expected.ToolPositionMetres) <= .00001f &&
                RotationDifferenceDegrees(tcp.localRotation, expected.ToolRotation) <= .005f;
        }

        private static bool Near(Vector3 a, Vector3 b, float tolerance) => Vector3.Distance(a, b) <= tolerance;
        private static bool NearRotation(Quaternion a, Quaternion b) => RotationDifferenceDegrees(a, b) <= .005f;

        private static bool SameGeometry(FanucKinematicGeometry a, FanucKinematicGeometry b)
        {
            if (a?.Joints == null || b?.Joints == null || a.Joints.Count != 6 || b.Joints.Count != 6 ||
                !a.ToolPositionMetres.Equals(b.ToolPositionMetres) || !a.ToolRotation.Equals(b.ToolRotation)) return false;
            for (int i = 0; i < 6; i++)
            {
                var x = a.Joints[i]; var y = b.Joints[i];
                if (x == null || y == null || !x.PositionMetres.Equals(y.PositionMetres) ||
                    !x.Rotation.Equals(y.Rotation) || !x.LocalRotationAxis.Equals(y.LocalRotationAxis)) return false;
            }
            return true;
        }

        private static float RotationDifferenceDegrees(Quaternion a, Quaternion b)
        {
            var delta = Quaternion.Inverse(a) * b;
            return (float)(2 * Math.Atan2(Math.Sqrt((double)delta.x * delta.x + (double)delta.y * delta.y +
                (double)delta.z * delta.z), Math.Abs(delta.w)) * 180 / Math.PI);
        }

        public bool MoveJoint(int index, float degrees)
        {
            if (!Application.isPlaying || !IsReady || index < 0 || index >= joints.Length || !joints[index].CanMoveTo(degrees)) return false;
            if (activeMotion != null) StopAll();
            LastMotionError = null;
            return joints[index].MoveTo(degrees);
        }

        public void StopJoint(int index)
        {
            if (!Application.isPlaying || index < 0 || index >= joints.Length) return;
            if (activeMotion != null) { StopAll(); return; }
            if (joints[index] != null) joints[index].Stop();
        }

        public void HomeAll()
        {
            if (!Application.isPlaying || !IsReady) return;
            foreach (var joint in joints) if (!joint.CanMoveTo(0f)) return;
            if (activeMotion != null) StopAll();
            LastMotionError = null;
            foreach (var joint in joints) joint.MoveTo(0f);
        }

        public void StopAll()
        {
            if (!Application.isPlaying) return;
            var previous = activeMotion;
            activeMotion = null;
            if (previous != null)
                foreach (var joint in previous.Axes) if (joint != null) joint.Stop();
            foreach (var joint in joints)
                if (joint != null) joint.Stop();
        }

        private void OnDisable()
        {
            StopAll();
        }
    }
}
