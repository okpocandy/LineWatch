using System;
using System.Collections.Generic;
using UnityEngine;
using SIK = LineWatch.ScaraInverseKinematics;
using FIK = LineWatch.FanucInverseKinematics;

namespace LineWatch
{
    public sealed class RobotWaypointSequence : MonoBehaviour
    {
        public enum RunState { Idle, Running, Paused, Faulted, Completed, Acting, Cancelled }
        [SerializeField] private RobotWaypointSet waypoints;
        [SerializeField] private ScaraJointLab scara;
        [SerializeField] private FanucJointLab fanuc;

        private RobotWaypointSet source;
        private CellCommandGate cellGate;
        private void Awake() { cellGate = new CellCommandGate(this); }
        private ScaraJointLab scaraOwner;
        private FanucJointLab fanucOwner;
        private Pose[] targets = new Pose[4];
        private LineWatchCellLayout transferLayout;
        private RobotWaypointSet transferDropSource;
        private RobotWaypointSet transferPickSource;
        private CellRunController transferCoordinator;
        private CellProductTracker transferTracker;
        private ProductDestination transferDestination;
        private Transform transferInput, transferDrop;
        private Pose transferInputPose, transferDropPose;
        private uint ownedVersion;
        private ScaraForwardKinematics.Geometry scaraGeometry;
        private RobotGripper transportGripper;
        private GrippableItem transportItem;
        private string cycleProductId, programName;
        private readonly List<ProgramRecord> cancellations = new List<ProgramRecord>();
        private readonly List<ProgramRecord> scenarioResets = new List<ProgramRecord>();

        // Immutable evidence survives a later, separately requested program start.
        public sealed class ProgramRecord
        {
            public string Reason { get; }
            public string ProgramName { get; }
            public string ProductId { get; }
            public RunState PreviousState { get; }
            public int StepIndex { get; }
            public int LastArrivedStepIndex { get; }
            public Pose LastArrivalPose { get; }
            public Pose LastReleasePose { get; }
            public bool IsPickPlace { get; }
            public bool IsCellTransfer { get; }
            public bool IsFanucTransfer { get; }
            public ProductDestination? FanucDestination { get; }
            public int GripCount { get; }
            public int ReleaseCount { get; }
            public string FaultReason { get; }
            public uint ProgramRevision { get; }
            public IReadOnlyList<Pose> Targets { get; }

            internal ProgramRecord(RobotWaypointSequence sequence, string reason)
            {
                Reason = reason; ProgramName = sequence.programName; ProductId = sequence.cycleProductId;
                PreviousState = sequence.State; StepIndex = sequence.StepIndex;
                LastArrivedStepIndex = sequence.LastArrivedStepIndex; LastArrivalPose = sequence.LastArrivalPose;
                LastReleasePose = sequence.LastReleasePose; IsPickPlace = sequence.IsPickPlace;
                IsCellTransfer = sequence.IsCellTransfer;
                IsFanucTransfer = sequence.IsFanucTransfer;
                FanucDestination = sequence.FanucDestination;
                GripCount = sequence.GripCount; ReleaseCount = sequence.ReleaseCount; FaultReason = sequence.LastError;
                ProgramRevision = sequence.ProgramRevision;
                Targets = Array.AsReadOnly((Pose[])sequence.targets.Clone());
            }
        }

        public ProgramRecord[] CancellationSnapshot() => cancellations.ToArray();
        public ProgramRecord[] ScenarioResetSnapshot() => scenarioResets.ToArray();

        public RobotWaypointSet Waypoints => waypoints;
        public uint ProgramRevision { get; private set; }
        public RunState State { get; private set; }
        public int StepIndex { get; private set; }
        public string LastError { get; private set; }
        public string LastAcknowledgedFaultReason { get; private set; }
        public int LastArrivedStepIndex { get; private set; } = -1;
        public Pose LastArrivalPose { get; private set; }
        public bool IsBusy => State == RunState.Running || State == RunState.Acting;
        public bool HasPendingProgram => State != RunState.Idle && State != RunState.Completed && State != RunState.Cancelled;
        public bool IsPickPlace { get; private set; }
        public bool IsCellTransfer { get; private set; }
        public bool IsFanucTransfer => IsCellTransfer && fanucOwner != null;
        public ProductDestination? FanucDestination => IsFanucTransfer ? transferDestination : (ProductDestination?)null;
        public int StepCount => targets.Length;
        private int ReleaseStep => IsCellTransfer ? 4 : 3;
        public GrippableItem CycleItem => transportItem;
        public int GripCount { get; private set; }
        public int ReleaseCount { get; private set; }
        public Pose LastReleasePose { get; private set; }
        public string StepName => IsFanucTransfer
            ? StepIndex == 0 ? "인계 접근" : StepIndex == 1 ? "인계 집기" : StepIndex == 2 ? "인계 상승" :
                StepIndex == 3 ? $"{transferDestination} 출구 접근" : StepIndex == 4 ? $"{transferDestination} 출구 놓기" : StepIndex == 5 ? $"{transferDestination} 출구 상승" : StepIndex == 6 ? $"{transferDestination} 출구 퇴피" : "완료"
            : IsCellTransfer
            ? StepIndex == 0 ? "투입 접근" : StepIndex == 1 ? "투입 집기" : StepIndex == 2 ? "투입 상승" :
                StepIndex == 3 ? "인계 접근" : StepIndex == 4 ? "인계 놓기" : StepIndex == 5 ? "인계 상승" : StepIndex == 6 ? "인계 퇴피" : "완료"
            : StepIndex == 0 ? "접근" : StepIndex == 1 ? "작업" : StepIndex == 2 ? "상승" : StepIndex == 3 ? "퇴피" : "완료";
        public bool IsReady => isActiveAndEnabled && waypoints != null &&
            ((scara != null && fanuc == null && scara.IsReady && waypoints.Robot == RobotWaypointSet.RobotType.Scara) ||
             (fanuc != null && scara == null && fanuc.IsReady && waypoints.Robot == RobotWaypointSet.RobotType.Fanuc));

        public bool TryStart(out string error) => StartProgram(null, null, out error);
        public bool TryStartPickPlace(RobotGripper gripper, GrippableItem item, out string error)
        {
            if (gripper == null || item == null) { error = "그리퍼와 준비된 시험 물체를 확인하세요."; return false; }
            return StartProgram(gripper, item, out error);
        }
        private bool StartProgram(RobotGripper gripper, GrippableItem item, out string error)
        {
            if (!CanRequest(out error)) return false;
            if (IsBusy) { error = "실행 중입니다. 정지 후 다시 시작하세요."; return false; }
            if (IsPickPlace && transportGripper != null && transportGripper.HeldItem != null)
            { error = "보유 물체를 놓거나 기존 운전을 재개하세요."; return false; }
            if (!AtRest(scara, fanuc)) { error = "수동 이동이 끝나거나 정지한 뒤 시작하세요."; return false; }
            if (!waypoints.TryValidate(out error)) return false;
            var candidate = ReadTargets(waypoints);
            if (gripper != null)
            {
                if (gripper.gameObject != gameObject || gripper.Scara != scara || gripper.Fanuc != fanuc)
                { error = "이 순차 운전과 같은 로봇의 그리퍼가 필요합니다."; return false; }
                if (!gripper.TryCheckPlannedGrip(item, candidate[1], out error)) return false;
            }
            if (!Preflight(scara, fanuc, candidate, 0, out error)) return false;
            if (!Dispatch(scara, fanuc, candidate[0], 0, out error)) return false;
            source = waypoints; scaraOwner = scara; fanucOwner = fanuc;
            ProgramRevision++;
            targets = candidate;
            if (scaraOwner != null) scaraGeometry = scaraOwner.Geometry;
            ownedVersion = Version;
            StepIndex = 0; LastError = null; State = RunState.Running;
            LastArrivedStepIndex = -1;
            IsPickPlace = gripper != null; transportGripper = gripper; transportItem = item;
            IsCellTransfer = false; transferLayout = null; transferDropSource = null; transferInput = null; transferDrop = null;
            transferPickSource = null; transferCoordinator = null; transferTracker = null;
            cycleProductId = item != null ? item.ProductId : null; programName = source.name;
            GripCount = 0; ReleaseCount = 0;
            return true;
        }

        internal bool TryPrepareScaraTransfer(CellRunController owner, LineWatchCellLayout layout, RobotGripper gripper, GrippableItem item, out string error)
        {
            if (!CanReadRequest(out error)) return false;
            if (!cellGate.IsManagedBy(owner) || !(owner.State == CellRunState.Stopped && !owner.HasControlledSequence || owner.AllowsAutomaticInputPreparation(this)) ||
                layout == null || layout.GetComponent<CellRunController>() != owner || layout.Scara != scara || fanuc != null ||
                HasPendingProgram || !AtRest(scara, null))
            { error = "같은 정지 셀의 빈 SCARA 경로만 준비할 수 있습니다."; return false; }
            var drop = layout.ScaraHandoffWaypoints;
            if (layout.InputStop == null || layout.HandoffPoint == null || drop == null || drop.Robot != RobotWaypointSet.RobotType.Scara ||
                gripper == null || gripper.gameObject != gameObject || gripper.Scara != scara || gripper.Fanuc != null ||
                item == null || !item.IsReady || item.ProductTracker != layout.GetComponent<CellProductTracker>())
            { error = "투입/인계 지점·SCARA·등록된 원래 상품 연결을 확인하세요."; return false; }
            if (!waypoints.TryValidate(out error) || !drop.TryValidate(out error)) return false;
            var pickWork = StationWork(gripper, item, layout.InputStop);
            var dropWork = StationWork(gripper, item, layout.HandoffPoint);
            if (Vector3.Distance(dropWork.position, drop.Work.PositionMetres) > .00001f || Angle(dropWork.rotation, drop.Work.Rotation) > .005f)
            { error = "인계 경유점과 실제 상품/집게의 인계 기준이 일치하지 않습니다."; return false; }
            var lift = drop.Approach.PositionMetres - drop.Work.PositionMetres;
            if (lift.y <= 0 || Mathf.Abs(lift.x) > .00001f || Mathf.Abs(lift.z) > .00001f)
            { error = "인계 접근점은 작업점 바로 위에 있어야 합니다."; return false; }
            var pickApproach = new Pose(pickWork.position + lift, pickWork.rotation);
            var candidate = new[] { pickApproach, pickWork, pickApproach, new Pose(drop.Approach.PositionMetres, drop.Approach.Rotation),
                new Pose(drop.Work.PositionMetres, drop.Work.Rotation), new Pose(drop.Approach.PositionMetres, drop.Approach.Rotation),
                new Pose(drop.Retreat.PositionMetres, drop.Retreat.Rotation) };
            if (!gripper.TryCheckPlannedGrip(item, pickWork, out error) || !Preflight(scara, null, candidate, 0, out error)) return false;
            // Commit only after every product, grip and path check. Preparation emits no command.
            targets = candidate; source = waypoints; scaraOwner = scara; fanucOwner = null;
            scaraGeometry = scara.Geometry; ownedVersion = Version;
            IsPickPlace = true; IsCellTransfer = true; transportGripper = gripper; transportItem = item;
            cycleProductId = item.ProductId; programName = "SCARA 투입→인계";
            transferLayout = layout; transferDropSource = drop; transferInput = layout.InputStop; transferDrop = layout.HandoffPoint;
            transferPickSource = null; transferCoordinator = owner; transferTracker = item.ProductTracker;
            transferInputPose = new Pose(transferInput.position, transferInput.rotation); transferDropPose = new Pose(transferDrop.position, transferDrop.rotation);
            StepIndex = 0; LastArrivedStepIndex = -1; LastError = null; GripCount = 0; ReleaseCount = 0;
            LastArrivalPose = default; LastReleasePose = default; State = RunState.Paused; ProgramRevision++;
            return true;
        }

        internal bool TryPrepareFanucTransfer(CellRunController owner, LineWatchCellLayout layout, RobotGripper gripper, GrippableItem item, ProductDestination destination, out string error)
        {
            if (!CanReadRequest(out error)) return false;
            if (!cellGate.IsManagedBy(owner) || !(owner.State == CellRunState.Stopped && !owner.HasControlledSequence || owner.AllowsAutomaticHandoffPreparation(this)) ||
                layout == null || layout.GetComponent<CellRunController>() != owner || layout.Fanuc != fanuc || scara != null ||
                HasPendingProgram || !AtRest(null, fanuc))
            { error = "같은 정지 셀의 빈 FANUC 경로만 준비할 수 있습니다."; return false; }
            var pick = layout.FanucHandoffWaypoints;
            var output = destination == ProductDestination.A ? layout.OutputA : destination == ProductDestination.B ? layout.OutputB : null;
            if (layout.HandoffPoint == null || output == null || pick == null || pick.Robot != RobotWaypointSet.RobotType.Fanuc ||
                gripper == null || !gripper.IsReady || gripper.gameObject != gameObject || gripper.Fanuc != fanuc || gripper.Scara != null ||
                item == null || !item.IsReady || item.ProductTracker != layout.GetComponent<CellProductTracker>())
            { error = "인계/출구·FANUC·등록된 원래 상품 연결을 확인하세요."; return false; }
            if (!waypoints.TryValidate(out error) || !pick.TryValidate(out error) || !owner.TryCheckScaraRetreat(out error)) return false;
            var pickWork = StationWork(gripper, item, layout.HandoffPoint);
            if (Vector3.Distance(pickWork.position, pick.Work.PositionMetres) > .00001f || Angle(pickWork.rotation, pick.Work.Rotation) > .005f)
            { error = "인계 경유점과 원래 상품/집게의 집기 기준이 다릅니다."; return false; }
            var dropWork = StationWork(gripper, item, output);
            var lift = pick.Approach.PositionMetres - pick.Work.PositionMetres;
            var approach = new Pose(dropWork.position + lift, dropWork.rotation);
            var retreat = new Pose(dropWork.position + (pick.Retreat.PositionMetres - pick.Work.PositionMetres), dropWork.rotation);
            var candidate = new[] { new Pose(pick.Approach.PositionMetres, pick.Approach.Rotation), new Pose(pick.Work.PositionMetres, pick.Work.Rotation),
                new Pose(pick.Approach.PositionMetres, pick.Approach.Rotation), approach, dropWork, approach, retreat };
            if (!gripper.TryCheckPlannedGrip(item, candidate[1], out error) || !Preflight(null, fanuc, candidate, 0, out error)) return false;
            targets = candidate; source = waypoints; scaraOwner = null; fanucOwner = fanuc; ownedVersion = Version;
            IsPickPlace = true; IsCellTransfer = true; transportGripper = gripper; transportItem = item;
            cycleProductId = item.ProductId; transferDestination = destination; programName = $"FANUC 인계→{destination}";
            transferLayout = layout; transferDropSource = null; transferPickSource = pick; transferCoordinator = owner; transferTracker = item.ProductTracker;
            transferInput = layout.HandoffPoint; transferDrop = output;
            transferInputPose = new Pose(transferInput.position, transferInput.rotation); transferDropPose = new Pose(transferDrop.position, transferDrop.rotation);
            StepIndex = 0; LastArrivedStepIndex = -1; LastError = null; GripCount = 0; ReleaseCount = 0;
            LastArrivalPose = default; LastReleasePose = default; State = RunState.Paused; ProgramRevision++;
            return true;
        }

        private static Pose StationWork(RobotGripper gripper, GrippableItem item, Transform station)
        {
            var point = item.GetGripPoint(gripper.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc);
            var pointOffset = item.transform.InverseTransformPoint(point.position);
            var pointRotation = Quaternion.Inverse(item.transform.rotation) * point.rotation;
            var rotation = station.rotation * pointRotation * Quaternion.Inverse(Quaternion.Inverse(gripper.Tcp.rotation) * gripper.Contact.rotation);
            var position = station.position + station.rotation * pointOffset - rotation * gripper.Tcp.InverseTransformPoint(gripper.Contact.position);
            return new Pose(gripper.RobotFrame.InverseTransformPoint(position), Quaternion.Inverse(gripper.RobotFrame.rotation) * rotation);
        }

        public bool TryResume(out string error)
        {
            if (!CanRequest(out error)) return false;
            return Resume(out error);
        }

        internal bool TryResumeFromCell(CellRunController owner, out string error)
        {
            if (owner == null || owner.ControlledSequence != this || !cellGate.IsManagedBy(owner))
            { error = "이 경로를 인계받은 셀 관리자만 재개할 수 있습니다."; return false; }
            if (!cellGate.CanIssue(this, out error)) return false;
            return Resume(out error);
        }

        // Read-only preparation. Cell Start/Resume checks this before changing its state;
        // execution validates again against the latest joints/program before dispatch.
        internal bool TryValidateResume(out string error)
        {
            if (!CanReadRequest(out error)) return false;
            if (State != RunState.Paused && State != RunState.Faulted) { error = "중단된 순서가 없습니다."; return false; }
            if (!SameProgram()) { error = "경유점이나 로봇 연결이 바뀌었습니다. 처음부터 시작하세요."; return false; }
            if (!AtRest(scaraOwner, fanucOwner)) { error = "현재 수동 이동을 정지한 뒤 재개하세요."; return false; }
            if (!CheckTransport(out error)) return false;
            if (IsPickPlace && GripCount == 0 && !transportGripper.TryCheckPlannedGrip(transportItem, targets[1], out error)) return false;
            if (!Preflight(scaraOwner, fanucOwner, targets, StepIndex, out error)) return false;
            return true;
        }

        // Cell recovery preserves the saved step and action counts. A pick/place program
        // can be acknowledged before its grip or while the same item is still held.
        internal bool CanAcknowledgeFault(CellRunController owner, out string error)
        {
            if (owner == null || owner.ControlledSequence != this)
            { error = "고장 경로를 복구 대상으로 먼저 인계하세요."; return false; }
            return CanPrepareFaultRecovery(owner, out error);
        }

        // Read-only eligibility before command ownership is assigned. The same transport
        // checks are used again for acknowledgement; neither query claims a physical item.
        internal bool CanPrepareFaultRecovery(CellRunController owner, out string error)
        {
            if (!CanReadRequest(out error)) return false;
            if (State != RunState.Faulted || owner == null || !owner.isActiveAndEnabled || !owner.IsFaulted ||
                !cellGate.IsManagedBy(owner))
            { error = "같은 활성 셀의 고장 경로만 복구 대상으로 준비할 수 있습니다."; return false; }
            if (IsPickPlace)
            {
                bool beforeGrip = GripCount == 0 && ReleaseCount == 0 && StepIndex >= 0 && StepIndex <= 1;
                bool holding = GripCount == 1 && ReleaseCount == 0 && StepIndex >= 1 && StepIndex <= ReleaseStep;
                if (!beforeGrip && !holding)
                { error = "저장 단계와 집기/놓기 횟수가 복구 범위와 맞지 않습니다. 놓기 완료 후 복구는 별도 지원이 필요합니다."; return false; }
                if (!CheckTransport(out error)) return false;
            }
            if (!SameProgram()) { error = "저장된 경유점과 로봇 연결을 원상 복원하세요."; return false; }
            return true;
        }

        // Only the coordinator calls this after all paths/FK and the final tracker check
        // succeed. No callbacks, dispatch or action counters change during the commits.
        internal void AcknowledgeFaultFromCell()
        {
            LastAcknowledgedFaultReason = LastError;
            LastError = null;
            State = RunState.Paused;
        }

        // The coordinator has checked stopped robots and the explicit fault-only request.
        // Retain the saved program and physical holding; never dispatch, release or acknowledge.
        internal void CancelFromCell(string reason)
        {
            cancellations.Add(new ProgramRecord(this, reason));
            State = RunState.Cancelled;
        }

        internal void ResetScenarioFromCell(string reason)
        {
            scenarioResets.Add(new ProgramRecord(this, reason));
            source = null; scaraOwner = null; fanucOwner = null;
            transportGripper = null; transportItem = null; cycleProductId = null; programName = null;
            targets = new Pose[4]; ownedVersion = 0; scaraGeometry = default;
            IsCellTransfer = false; transferLayout = null; transferDropSource = null; transferInput = null; transferDrop = null;
            transferPickSource = null; transferCoordinator = null; transferTracker = null;
            State = RunState.Idle; StepIndex = 0; LastArrivedStepIndex = -1;
            LastArrivalPose = default; LastReleasePose = default; IsPickPlace = false;
            GripCount = 0; ReleaseCount = 0; LastError = null; ProgramRevision++;
        }

        private bool Resume(out string error)
        {
            if (!TryValidateResume(out error)) return false;
            if (!Dispatch(scaraOwner, fanucOwner, targets[StepIndex], StepIndex, out error)) return false;
            if (scaraOwner != null) scaraGeometry = scaraOwner.Geometry;
            ownedVersion = Version; LastError = null; State = RunState.Running;
            return true;
        }

        public void Pause()
        {
            if (!Application.isPlaying || !IsBusy) return;
            StopOwned(); State = RunState.Paused; LastError = null;
        }

        private void Update()
        {
            if (State != RunState.Running) return;
            if (!SameProgram() || !IsReady) { Fault("경유점·로봇 연결·활성 상태가 바뀌었습니다."); return; }
            if (!CheckTransport(out var transportError)) { Fault(transportError); return; }
            if (Version != ownedVersion)
            {
                if (!string.IsNullOrEmpty(MotionError)) { Fault(MotionError); return; }
                State = RunState.Paused;
                LastError = "수동/외부 명령으로 중단했습니다. 재개 전 현재 자세를 확인하세요.";
                return; // The newer command owns the robot; do not stop or overwrite it.
            }
            if (scaraOwner != null && !scaraOwner.Geometry.Equals(scaraGeometry)) { Fault("SCARA 기하 입력이 바뀌었습니다."); return; }
            if (!string.IsNullOrEmpty(MotionError)) { Fault(MotionError); return; }
            if (!ModelAligned(out var modelError)) { Fault(modelError); return; }
            if (!ClockRunning() || Moving || !AtRest(scaraOwner, fanucOwner)) return;
            if (!AtTarget(out var actual, out var error)) { Fault(error); return; }
            LastArrivedStepIndex = StepIndex; LastArrivalPose = actual;
            if (IsPickPlace && !PerformAction(out error)) { Fault(error); return; }
            if (State != RunState.Running) return;
            if (StepIndex == StepCount - 1) { StepIndex = StepCount; State = RunState.Completed; return; }
            int next = StepIndex + 1;
            // Advance the logical step only after a fresh move request succeeds.
            if (!Dispatch(scaraOwner, fanucOwner, targets[next], next, out error)) { Fault(error); return; }
            StepIndex = next; ownedVersion = Version;
        }

        private bool CanRequest(out string error)
        {
            if (!cellGate.CanIssue(null, out error)) return false;
            return CanReadRequest(out error);
        }

        private bool CanReadRequest(out string error)
        {
            error = null;
            if (!Application.isPlaying || !IsReady) { error = "Play 상태와 경유점·로봇 연결을 확인하세요."; return false; }
            if (!ClockRunning()) { error = "일시정지와 시간 배율을 해제한 뒤 요청하세요."; return false; }
            return true;
        }

        private static bool ClockRunning()
        {
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPaused) return false;
#endif
            return Time.timeScale > 0f;
        }

        private bool SameProgram()
        {
            if (source == null || source != waypoints || scara != scaraOwner || fanuc != fanucOwner || !source.TryValidate(out _)) return false;
            if (IsFanucTransfer)
                return transferLayout != null && transferLayout.Fanuc == fanucOwner && transferLayout.HandoffPoint == transferInput &&
                    (transferDestination == ProductDestination.A ? transferLayout.OutputA : transferLayout.OutputB) == transferDrop && transferLayout.FanucHandoffWaypoints == transferPickSource &&
                    transferInput != null && transferDrop != null && transferPickSource != null && transferPickSource.Robot == RobotWaypointSet.RobotType.Fanuc && transferPickSource.TryValidate(out _) &&
                    transferInput.position.Equals(transferInputPose.position) && transferInput.rotation.Equals(transferInputPose.rotation) &&
                    transferDrop.position.Equals(transferDropPose.position) && transferDrop.rotation.Equals(transferDropPose.rotation) &&
                    Same(targets[0], transferPickSource.Approach) && Same(targets[1], transferPickSource.Work) &&
                    targets[6].position.Equals(targets[4].position + (transferPickSource.Retreat.PositionMetres - transferPickSource.Work.PositionMetres));
            if (IsCellTransfer)
                return transferLayout != null && transferLayout.Scara == scaraOwner && transferLayout.InputStop == transferInput &&
                    transferLayout.HandoffPoint == transferDrop && transferLayout.ScaraHandoffWaypoints == transferDropSource &&
                    transferInput != null && transferDrop != null && transferDropSource != null && transferDropSource.TryValidate(out _) &&
                    transferInput.position.Equals(transferInputPose.position) && transferInput.rotation.Equals(transferInputPose.rotation) &&
                    transferDrop.position.Equals(transferDropPose.position) && transferDrop.rotation.Equals(transferDropPose.rotation) &&
                    Same(targets[3], transferDropSource.Approach) && Same(targets[4], transferDropSource.Work) && Same(targets[6], transferDropSource.Retreat);
            return Same(targets[0], source.Approach) && Same(targets[1], source.Work) && Same(targets[3], source.Retreat);
        }
        private static bool Same(Pose pose, RobotWaypointSet.Target target) => pose.position.Equals(target.PositionMetres) && pose.rotation.Equals(target.Rotation);
        private static Pose[] ReadTargets(RobotWaypointSet set) => new[] { new Pose(set.Approach.PositionMetres, set.Approach.Rotation),
            new Pose(set.Work.PositionMetres, set.Work.Rotation), new Pose(set.Approach.PositionMetres, set.Approach.Rotation), new Pose(set.Retreat.PositionMetres, set.Retreat.Rotation) };
        private uint Version => scaraOwner != null ? scaraOwner.CommandVersion : fanucOwner != null ? fanucOwner.CommandVersion : 0;
        private bool Moving => scaraOwner != null ? scaraOwner.IsTrajectoryMoving : fanucOwner != null && fanucOwner.IsTcpMoving;
        private string MotionError => scaraOwner != null ? scaraOwner.LastMotionError : fanucOwner != null ? fanucOwner.LastMotionError : null;

        private static bool AtRest(ScaraJointLab s, FanucJointLab f)
        {
            if (s != null) return !s.IsTrajectoryMoving && !s.Axis1.IsMoving && !s.Axis2.IsMoving && !s.Axis3.IsMoving && !s.Axis4.IsMoving;
            if (f == null || f.IsTcpMoving) return false;
            foreach (var joint in f.Joints) if (joint == null || joint.IsMoving) return false;
            return true;
        }

        private void StopOwned()
        {
            if (Version != ownedVersion) return;
            if (scaraOwner != null) scaraOwner.StopAll();
            if (fanucOwner != null) fanucOwner.StopAll();
        }
        private void Fault(string error) { StopOwned(); State = RunState.Faulted; LastError = error; }
        private void OnDisable() { if (IsBusy) { StopOwned(); State = RunState.Paused; } }

        private bool CheckTransport(out string error)
        {
            error = null;
            if (!IsPickPlace) return true;
            if (transportGripper == null || !transportGripper.IsReady || transportGripper.Scara != scaraOwner || transportGripper.Fanuc != fanucOwner ||
                transportItem == null || !transportItem.IsReady)
                error = "운반 물체나 그리퍼가 유효하지 않습니다. 상태를 확인하고 새 운전을 시작하세요.";
            else if (GripCount == 0 && (transportGripper.HeldItem != null || transportItem.Owner != null))
                error = "집기 전 물체/그리퍼의 보유자가 바뀌었습니다.";
            else if (GripCount > ReleaseCount && (transportGripper.HeldItem != transportItem || transportItem.Owner != transportGripper))
                error = "운반 물체의 보유가 해제되거나 바뀌었습니다. 자동 재집기하지 않습니다.";
            if (error == null && IsCellTransfer)
            {
                var tracker = transferLayout != null ? transferTracker : null;
                if (tracker == null || transportItem.ProductTracker != tracker || transportItem.ProductId != cycleProductId ||
                    !tracker.TryGetItem(cycleProductId, out var original) || original != transportItem || !tracker.TryGet(cycleProductId, out var record))
                    error = "저장한 등록 상품과 원래 실물 연결이 다릅니다.";
                else
                {
                    var expected = IsFanucTransfer
                        ? GripCount == 0 ? ProductLocation.Handoff : ReleaseCount == 0 ? ProductLocation.Fanuc : transferDestination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB
                        : GripCount == 0 ? ProductLocation.Input : ReleaseCount == 0 ? ProductLocation.Scara : ProductLocation.Handoff;
                    bool occupied = IsFanucTransfer ? GripCount > 0 && tracker.HandoffProductId != null : ReleaseCount == 0 && tracker.HandoffProductId != null;
                    if (record.Location != expected || occupied || IsFanucTransfer && record.Destination != transferDestination ||
                        ReleaseCount > 0 && (transportItem.Owner != null || transportGripper.HeldItem != null))
                        error = "상품의 운반 단계 또는 인계대 점유/실제 보유가 맞지 않습니다.";
                    else if (!tracker.TryCheckPhysicalState(out error)) return false;
                    else if (IsFanucTransfer && (transferCoordinator == null || !transferCoordinator.TryCheckScaraRetreat(out error)))
                    { if (error == null) error = "원래 셀 운전 관리자 연결을 확인하세요."; return false; }
                }
            }
            return error == null;
        }

        internal bool AllowsGripperAction(RobotGripper gripper, bool releasing) => IsPickPlace && State == RunState.Acting &&
            gripper == transportGripper && Version == ownedVersion &&
            (releasing ? StepIndex == ReleaseStep && GripCount == 1 && ReleaseCount == 0 : StepIndex == 1 && GripCount == 0);

        private bool PerformAction(out string error)
        {
            error = null;
            if (StepIndex == 1 && GripCount == 0)
            {
                State = RunState.Acting;
                if (!transportGripper.TryGripFromSequence(transportItem, this, out error)) return false;
                GripCount++;
                if (State == RunState.Acting) State = RunState.Running;
            }
            if (StepIndex == ReleaseStep && ReleaseCount == 0)
            {
                State = RunState.Acting;
                if (!transportGripper.TryReleaseFromSequence(this, out error)) return false;
                LastReleasePose = new Pose(transportItem.Body.position, transportItem.Body.rotation);
                ReleaseCount++;
                if (State == RunState.Acting) State = RunState.Running;
            }
            return true;
        }

        private bool AtTarget(out Pose actual, out string error)
        {
            var target = targets[StepIndex]; error = null; actual = default;
            if (scaraOwner != null)
            {
                if (!scaraOwner.TryReadFkDiagnostics(out var d, out error)) return false;
                actual = new Pose(d.ActualPositionMetres, Quaternion.Euler(0, d.ActualYawDegrees, 0));
                if (d.PositionErrorMetres > .00001f || d.YawErrorDegrees > .005f ||
                    Vector3.Distance(d.ActualPositionMetres, target.position) > .002f || Mathf.Abs(Mathf.DeltaAngle(d.ActualYawDegrees, Yaw(target.rotation))) > .5f)
                    error = "SCARA 실제 TCP가 목표/계산과 일치하지 않습니다.";
            }
            else
            {
                if (!fanucOwner.TryReadFkDiagnostics(out var d, out error)) return false;
                actual = new Pose(d.ActualPositionMetres, d.ActualRotation);
                if (d.PositionErrorMetres > .00001f || d.RotationErrorDegrees > .005f ||
                    Vector3.Distance(d.ActualPositionMetres, target.position) > .002f || Angle(d.ActualRotation, target.rotation) > .5f)
                    error = "FANUC 실제 TCP가 목표/계산과 일치하지 않습니다.";
            }
            return error == null;
        }

        private bool Dispatch(ScaraJointLab s, FanucJointLab f, Pose target, int index, out string error)
        {
            if (s != null) return s.TryMoveTcpFromSequence(target.position, Yaw(target.rotation), ScaraJointLab.IkSelection.PositiveElbow,
                index == 0 ? ScaraJointLab.MotionMode.Joint : ScaraJointLab.MotionMode.Linear, this, out _, out error);
            return f.TryMoveTcpFromSequence(target.position, target.rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, this, out _, out error);
        }

        private bool ModelAligned(out string error)
        {
            error = null;
            if (scaraOwner != null)
            {
                if (!scaraOwner.TryReadFkDiagnostics(out var d, out error)) return false;
                if (d.PositionErrorMetres > .00001f || d.YawErrorDegrees > .005f) error = "SCARA 이동 중 계산/실측이 다릅니다.";
            }
            else
            {
                if (!fanucOwner.TryReadFkDiagnostics(out var d, out error)) return false;
                if (d.PositionErrorMetres > .00001f || d.RotationErrorDegrees > .005f) error = "FANUC 이동 중 계산/실측이 다릅니다.";
            }
            return error == null;
        }

        private static bool Preflight(ScaraJointLab s, FanucJointLab f, Pose[] program, int first, out string error)
        {
            error = null;
            if (s != null)
            {
                if (!s.TryReadFkDiagnostics(out var d, out error)) return false;
                if (d.PositionErrorMetres > .00001f || d.YawErrorDegrees > .005f) { error = "SCARA 모델/기하 기준이 일치하지 않습니다."; return false; }
                var limits = new SIK.JointLimits(J(s.Axis1.MinimumDegrees, s.Axis2.MinimumDegrees, s.Axis3.MinimumMetres, s.Axis4.MinimumDegrees),
                    J(s.Axis1.MaximumDegrees, s.Axis2.MaximumDegrees, s.Axis3.MaximumMetres, s.Axis4.MaximumDegrees));
                var speeds = new Vector4(s.Axis1.SpeedDegreesPerSecond, s.Axis2.SpeedDegreesPerSecond, s.Axis3.SpeedMetresPerSecond, s.Axis4.SpeedDegreesPerSecond);
                var current = J(s.Axis1.CurrentPosition, s.Axis2.CurrentPosition, s.Axis3.CurrentPosition, s.Axis4.CurrentPosition);
                for (int i = first; i < program.Length; i++)
                {
                    var goal = program[i]; float yaw = Yaw(goal.rotation);
                    var result = SIK.Solve(goal.position, yaw, s.Geometry, limits);
                    bool found = false; double best = double.PositiveInfinity; SIK.Solution end = default;
                    foreach (var candidate in result.Solutions)
                    {
                        if (candidate.ElbowSign < 0) continue;
                        var q = candidate.Joints; double cost = Math.Pow(q.Q1Degrees - current.Q1Degrees, 2) +
                            Math.Pow(q.Q2Degrees - current.Q2Degrees, 2) + Math.Pow(q.Q4Degrees - current.Q4Degrees, 2);
                        if (cost < best) { found = true; best = cost; end = candidate; }
                    }
                    if (!result.Success || !found) { error = $"{i + 1}/{program.Length} 경유점 IK 사전 검사 실패: {result.Failure}"; return false; }
                    bool ok = i == 0 ? ScaraMotionPlanner.Joint(current, end, limits, speeds, out var plan, out var failure)
                        : ScaraMotionPlanner.Linear(current, goal.position, yaw, s.Geometry, limits, speeds, 1, out plan, out failure);
                    if (!ok) { error = $"{i + 1}/{program.Length} 경유점 경로 사전 검사 실패: {failure}"; return false; }
                    current = plan.End.Joints;
                }
            }
            else
            {
                if (!f.TryPreviewTcp(program[first].position, program[first].rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out var preview, out error)) return false;
                var current = new float[6]; var lower = new float[6]; var upper = new float[6];
                for (int i = 0; i < 6; i++)
                {
                    var axis = f.Joints[i];
                    if (float.IsNaN(axis.SpeedDegreesPerSecond) || float.IsInfinity(axis.SpeedDegreesPerSecond) || axis.SpeedDegreesPerSecond <= 0f)
                    { error = "FANUC의 양의 유한 속도를 확인하세요."; return false; }
                    current[i] = axis.CurrentPosition; lower[i] = axis.MinimumDegrees; upper[i] = axis.MaximumDegrees;
                }
                for (int i = first; i < program.Length; i++)
                {
                    var seeds = new List<float[]> { current, new float[6], new[] { 10f, -15f, 25f, 15f, 20f, -10f }, new[] { -10f, 15f, -25f, -15f, -20f, 10f } };
                    var result = i == first ? preview.Result : FIK.Solve(program[i].position, program[i].rotation, f.Geometry, lower, upper, seeds);
                    if (!result.Succeeded) { error = $"{i + 1}/{program.Length} 경유점 IK 사전 검사 실패: {result.Reason}"; return false; }
                    current = new float[6]; for (int j = 0; j < 6; j++) current[j] = result.JointDegrees[j];
                }
            }
            return true;
        }
        private static SIK.JointValues J(float a, float b, float c, float d) => new SIK.JointValues(a, b, c, d);
        private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
        private static float Angle(Quaternion a, Quaternion b)
        { var d = Quaternion.Inverse(a) * b; return (float)(2 * Math.Atan2(Math.Sqrt((double)d.x * d.x + (double)d.y * d.y + (double)d.z * d.z), Math.Abs(d.w)) * 180 / Math.PI); }
    }
}
