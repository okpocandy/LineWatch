using UnityEngine;

namespace LineWatch
{
    // Owns command permission and can connect two prepared robot programs for one product.
    // Product state and physical ownership remain with the tracker and grippers.
    [DefaultExecutionOrder(300)]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(LineWatchCellLayout))]
    public sealed class CellRunController : MonoBehaviour
    {
        public enum AutomaticTransferPhase { None, Scara, Fanuc, Completed, WaitingForInput }
        public enum AutomaticProgressKind { None, NormalInputSupply, RobotStep }
        public enum FaultSource { Cell, ProductTracker, Scara, Fanuc, InputSupply }
        public const float SupplyNoProgressTimeoutSeconds = 2f;
        public const float RobotNoProgressTimeoutSeconds = 2f;
        public readonly struct FaultRecord
        {
            public FaultSource Source { get; }
            public string EquipmentId { get; }
            public string OperationProductId { get; }
            public string CauseProductId { get; }
            public string Step { get; }
            public string Reason { get; }
            internal FaultRecord(FaultSource source, string equipmentId, string productId, string causeProductId, string step, string reason)
            { Source = source; EquipmentId = equipmentId; OperationProductId = productId; CauseProductId = causeProductId; Step = step; Reason = reason; }
        }
        private readonly CellRunStateMachine policy = new CellRunStateMachine();
        private LineWatchCellLayout layout;
        private CellProductTracker tracker;
        private ScaraJointLab scara;
        private FanucJointLab fanuc;
        private RobotGripper scaraGripper, fanucGripper;
        private RobotWaypointSequence scaraSequence, fanucSequence;
        private RobotWaypointSequence controlledSequence;
        private bool hasControlledSequence;
        private bool resumeDispatchInProgress;
        private bool automaticHandoffPreparation;
        private bool automaticInputPreparation;
        private ProductDestination automaticDestination;
        private AutomaticTransferPhase observedPhase;
        private uint observedProgramRevision;
        private int observedStepIndex = -1;
        private bool supplySampled;
        private float bestSupplyDistance, bestSupplyAngle;
        private uint observedRobotCommand;
        private double observedAppliedSeconds;

        public AutomaticTransferPhase AutomaticPhase { get; private set; }
        public string AutomaticProductId { get; private set; }
        public bool IsAutomaticContinuous { get; private set; }
        public ProductDestination? AutomaticDestination => AutomaticPhase == AutomaticTransferPhase.None ? (ProductDestination?)null : automaticDestination;
        public AutomaticProgressKind ProgressKind { get; private set; }
        public double ProgressActiveSeconds { get; private set; }
        public string ProgressProductId { get; private set; }
        public string ProgressEquipmentId { get; private set; }
        public string ProgressStep { get; private set; }
        public double SupplyNoProgressSeconds { get; private set; }
        public double RobotNoProgressSeconds { get; private set; }
        public bool IsProgressClockRunning => ProgressKind != AutomaticProgressKind.None && State == CellRunState.Running && ClockReady(out _);

        public CellRunState State => policy.State;
        public string FaultReason => policy.FaultReason;
        public string LastAcknowledgedFaultReason => policy.LastAcknowledgedFaultReason;
        public bool IsFaulted => policy.IsFaulted;
        public FaultRecord? ActiveFault { get; private set; }
        public FaultRecord? LastAcknowledgedFault { get; private set; }
        public bool HasControlledSequence => hasControlledSequence;
        public RobotWaypointSequence ControlledSequence => controlledSequence;

        private void Awake()
        {
            layout = GetComponent<LineWatchCellLayout>();
            tracker = GetComponent<CellProductTracker>();
            scara = layout.Scara; fanuc = layout.Fanuc;
            if (scara != null) { scaraGripper = scara.GetComponent<RobotGripper>(); scaraSequence = scara.GetComponent<RobotWaypointSequence>(); }
            if (fanuc != null) { fanucGripper = fanuc.GetComponent<RobotGripper>(); fanucSequence = fanuc.GetComponent<RobotWaypointSequence>(); }
            if (tracker != null) tracker.Faulted += OnTrackerFault;
        }

        public bool TryExecute(CellRunCommand command, out string error)
        {
            if (!Application.isPlaying) { error = "Play 상태에서 셀 명령을 요청하세요."; return false; }
            SynchronizeFault();
            if (!CanExecute(command, out error)) return false;
            if (command == CellRunCommand.Stop)
            {
                StopBoth();
                return policy.TryExecute(command, out error);
            }
            if (command == CellRunCommand.Reset)
            {
                // Validate everything before either latch changes. These synchronous
                // commits preserve data and cannot resume a path or emit a motor command.
                if (!ValidateRecovery(out error)) return false;
                if (!tracker.TryAcknowledgeFault(this, out error)) return false;
                if (hasControlledSequence && controlledSequence.State == RobotWaypointSequence.RunState.Faulted)
                    controlledSequence.AcknowledgeFaultFromCell();
                if (!policy.TryExecute(command, out error)) return false;
                LastAcknowledgedFault = ActiveFault; ActiveFault = null;
                ClearProgress();
                return true;
            }
            bool resumePath = false;
            if ((command == CellRunCommand.Start || command == CellRunCommand.Resume) && hasControlledSequence && controlledSequence.State != RobotWaypointSequence.RunState.Completed)
            {
                if (!controlledSequence.TryValidateResume(out error)) return false;
                resumePath = true;
            }
            if (!policy.TryExecute(command, out error)) return false;
            if (command == CellRunCommand.Pause) StopBoth();
            if (resumePath) return ResumeControlledPath(out error);
            return true;
        }

        private bool ResumeControlledPath(out string error)
        {
            // Admission re-reads cell health while the prepared path is still paused.
            resumeDispatchInProgress = true;
            try
            {
                if (controlledSequence.TryResumeFromCell(this, out error)) return true;
                Fault(error, controlledSequence == scaraSequence ? FaultSource.Scara : FaultSource.Fanuc); return false;
            }
            finally { resumeDispatchInProgress = false; }
        }

        public bool TryTakeControl(RobotWaypointSequence sequence, out string error)
        {
            if (!Application.isPlaying) { error = "Play 상태에서 셀 인계를 요청하세요."; return false; }
            SynchronizeFault();
            if (!CanTakeControl(sequence, out error)) return false;
            if (IsFaulted)
            {
                if (!ValidateRecoveryPrograms(out error)) return false;
            }
            else if (!sequence.TryValidateResume(out error)) return false;
            controlledSequence = sequence;
            hasControlledSequence = true;
            return true;
        }

        public bool CanPrepareScaraTransfer(out string error)
        {
            if (!CanReadReady(out error) || !ClockReady(out error)) return false;
            if (State != CellRunState.Stopped || !AtRest || hasControlledSequence ||
                HasPendingSequence(scaraSequence) || HasPendingSequence(fanucSequence))
            { error = "두 로봇을 정지하고 기존 인계/미완료 경로를 정리한 뒤 SCARA 상품 운반을 준비하세요."; return false; }
            if (!ReferenceEquals(scaraGripper.HeldItem, null) || !ReferenceEquals(fanucGripper.HeldItem, null))
            { error = "두 집게가 비어 있어야 합니다. 준비로 기존 보유를 변경하지 않습니다."; return false; }
            if (tracker.InputProductId == null || !tracker.TryGetItem(tracker.InputProductId, out _) || tracker.HandoffProductId != null)
            { error = "투입 상품 하나와 빈 인계대가 필요합니다."; return false; }
            return tracker.TryCheckPhysicalState(out error);
        }

        public bool TryPrepareScaraTransfer(out string error)
        {
            if (!CanPrepareScaraTransfer(out error)) return false;
            tracker.TryGet(tracker.InputProductId, out var current);
            return TryPrepareScaraTransfer(current, out error);
        }

        public bool CanPrepareAutomaticTransfer(out string error) => CanPrepareScaraTransfer(out error);

        public bool TryPrepareAutomaticTransfer(out string error)
        {
            if (!CanPrepareAutomaticTransfer(out error)) return false;
            tracker.TryGet(tracker.InputProductId, out var current);
            return TryPrepareAutomaticTransfer(current, out error);
        }

        public bool TryPrepareAutomaticTransfer(ProductRecord expected, out string error)
        {
            if (!TryPrepareScaraTransfer(expected, out error)) return false;
            IsAutomaticContinuous = false;
            AutomaticProductId = expected.Id; automaticDestination = expected.Destination;
            AutomaticPhase = AutomaticTransferPhase.Scara;
            return true;
        }

        public bool CanPrepareContinuousTransfer(out string error) => CanPrepareAutomaticTransfer(out error);

        public bool TryPrepareContinuousTransfer(out string error)
        {
            if (!CanPrepareContinuousTransfer(out error)) return false;
            tracker.TryGet(tracker.InputProductId, out var current);
            return TryPrepareContinuousTransfer(current, out error);
        }

        public bool TryPrepareContinuousTransfer(ProductRecord expected, out string error)
        {
            if (!TryPrepareAutomaticTransfer(expected, out error)) return false;
            IsAutomaticContinuous = true;
            return true;
        }

        public bool TryPrepareScaraTransfer(ProductRecord expected, out string error)
        {
            if (!CanPrepareScaraTransfer(out error)) return false;
            if (expected == null || expected.Id != tracker.InputProductId || !tracker.TryGet(expected.Id, out var current) || !ReferenceEquals(expected, current))
            { error = "표시 후 투입 상품 기록이 바뀌었습니다. 현재 상품을 확인한 뒤 다시 준비하세요."; return false; }
            tracker.TryGetItem(tracker.InputProductId, out var item);
            if (!scaraSequence.TryPrepareScaraTransfer(this, layout, scaraGripper, item, out error)) return false;
            controlledSequence = scaraSequence; hasControlledSequence = true;
            return true;
        }

        public bool CanPrepareFanucTransferToA(out string error)
            => CanPrepareFanucTransfer(ProductDestination.A, out error);

        public bool CanPrepareFanucTransfer(ProductDestination destination, out string error)
        {
            if (destination != ProductDestination.A && destination != ProductDestination.B)
            { error = "FANUC 운반 목적지는 A 또는 B여야 합니다."; return false; }
            if (!CanReadReady(out error) || !ClockReady(out error)) return false;
            if (State != CellRunState.Stopped || !AtRest || hasControlledSequence ||
                HasPendingSequence(scaraSequence) || HasPendingSequence(fanucSequence))
            { error = "두 로봇을 정지하고 기존 인계/미완료 경로를 정리한 뒤 FANUC 운반을 준비하세요."; return false; }
            if (!ReferenceEquals(scaraGripper.HeldItem, null) || !ReferenceEquals(fanucGripper.HeldItem, null))
            { error = "두 집게가 비어 있어야 합니다."; return false; }
            if (tracker.HandoffProductId == null || !tracker.TryGet(tracker.HandoffProductId, out var product) || product.Destination != destination)
            { error = $"인계대에 목적지 {destination}인 등록 상품이 필요합니다."; return false; }
            return tracker.TryCheckPhysicalState(out error) && TryCheckScaraRetreat(out error);
        }

        public bool TryPrepareFanucTransferToA(out string error)
            => TryPrepareFanucTransfer(ProductDestination.A, out error);

        public bool TryPrepareFanucTransfer(ProductDestination destination, out string error)
        {
            if (!CanPrepareFanucTransfer(destination, out error)) return false;
            tracker.TryGet(tracker.HandoffProductId, out var current);
            return TryPrepareFanucTransfer(destination, current, out error);
        }

        public bool TryPrepareFanucTransferToA(ProductRecord expected, out string error)
            => TryPrepareFanucTransfer(ProductDestination.A, expected, out error);

        public bool TryPrepareFanucTransfer(ProductDestination destination, ProductRecord expected, out string error)
        {
            if (!CanPrepareFanucTransfer(destination, out error)) return false;
            if (expected == null || expected.Id != tracker.HandoffProductId || !tracker.TryGet(expected.Id, out var current) || !ReferenceEquals(expected, current))
            { error = "표시 후 인계 상품 기록이 바뀌었습니다. 현재 상품을 다시 확인하세요."; return false; }
            tracker.TryGetItem(expected.Id, out var item);
            if (!fanucSequence.TryPrepareFanucTransfer(this, layout, fanucGripper, item, destination, out error)) return false;
            controlledSequence = fanucSequence; hasControlledSequence = true;
            return true;
        }

        internal bool TryCheckScaraRetreat(out string error)
        {
            error = null;
            var set = layout != null ? layout.ScaraHandoffWaypoints : null;
            if (scara == null || !scara.IsReady || set == null || set.Robot != RobotWaypointSet.RobotType.Scara)
            { error = "SCARA와 지정 퇴피 경유점 연결을 확인하세요."; return false; }
            if (!set.TryValidate(out error) || !scara.TryReadFkDiagnostics(out var d, out error)) return false;
            float yaw = Vector3.SignedAngle(Vector3.forward, set.Retreat.Rotation * Vector3.forward, Vector3.up);
            if (d.PositionErrorMetres > .00001f || d.YawErrorDegrees > .005f ||
                Vector3.Distance(d.ActualPositionMetres, set.Retreat.PositionMetres) > .002f || Mathf.Abs(Mathf.DeltaAngle(d.ActualYawDegrees, yaw)) > .5f)
            { error = "SCARA의 실제 TCP/FK와 지정 퇴피점을 확인하세요. 퇴피 전에는 FANUC 인계 운반을 허용하지 않습니다."; return false; }
            return true;
        }

        public bool TryReleaseControl(out string error)
        {
            if (!Ready(out error) || !CanReleaseControl(out error)) return false;
            controlledSequence = null;
            hasControlledSequence = false;
            ClearAutomaticTransfer();
            return true;
        }

        public bool CanCancelProgram(RobotWaypointSequence sequence, string reason, out string error)
            => CanCancelProgram(sequence, sequence != null ? sequence.ProgramRevision : 0, reason, out error);

        public bool CanCancelProgram(RobotWaypointSequence sequence, uint expectedRevision, string reason, out string error)
        {
            error = null;
            if (!Application.isPlaying || !isActiveAndEnabled || !IsFaulted || !Wired ||
                hasControlledSequence && controlledSequence == null)
            { error = "연결이 유효한 활성 Faulted 셀에서만 미완료 경로를 명시 종료합니다."; return false; }
            if (!ClockReady(out error)) return false;
            if (!AtRest) { error = "두 로봇과 순차 운전을 정지한 뒤 경로를 종료하세요."; return false; }
            if (sequence == null || sequence != scaraSequence && sequence != fanucSequence)
            { error = "이 셀의 종료 대상 경로를 선택하세요."; return false; }
            if (sequence.ProgramRevision != expectedRevision)
            { error = "선택 후 경로가 교체되거나 초기화됐습니다. 종료 대상을 다시 선택하세요."; return false; }
            if (sequence.State != RobotWaypointSequence.RunState.Paused && sequence.State != RobotWaypointSequence.RunState.Faulted)
            { error = "일시정지/고장으로 중단된 미완료 경로만 종료할 수 있습니다."; return false; }
            if (string.IsNullOrWhiteSpace(reason)) { error = "경로 종료 사유를 입력하세요."; return false; }
            return true;
        }

        public bool TryCancelProgram(RobotWaypointSequence sequence, string reason, out string error)
            => TryCancelProgram(sequence, sequence != null ? sequence.ProgramRevision : 0, reason, out error);

        public bool TryCancelProgram(RobotWaypointSequence sequence, uint expectedRevision, string reason, out string error)
        {
            if (!CanCancelProgram(sequence, expectedRevision, reason, out error)) return false;
            sequence.CancelFromCell(reason);
            if (hasControlledSequence && controlledSequence == sequence)
            {
                controlledSequence = null;
                hasControlledSequence = false;
                ClearAutomaticTransfer();
            }
            return true;
        }

        public bool CanResetScenario(string reason, out string error)
            => CanResetScenario(tracker != null ? tracker.ScenarioNumber : 0, reason, out error);

        public bool CanResetScenario(int expectedScenario, string reason, out string error)
        {
            if (!CanReadReady(out error) || !ClockReady(out error)) return false;
            if (tracker.ScenarioNumber != expectedScenario)
            { error = "시나리오가 바뀌었습니다. 새 사유를 입력해 대상을 다시 확인하세요."; return false; }
            if (State != CellRunState.Stopped || !AtRest)
            { error = "셀과 두 로봇을 정지한 뒤 시나리오를 초기화하세요."; return false; }
            if (HasPendingSequence(scaraSequence) || HasPendingSequence(fanucSequence))
            { error = "미완료 경로를 마치거나 고장 경로를 명시 종료하세요."; return false; }
            if (!ReferenceEquals(scaraGripper.HeldItem, null) || !ReferenceEquals(fanucGripper.HeldItem, null))
            { error = "두 집게가 비어 있어야 합니다. 초기화로 보유를 해제하지 않습니다."; return false; }
            if (string.IsNullOrWhiteSpace(reason)) { error = "시나리오 초기화 사유가 필요합니다."; return false; }
            return tracker.CanResetScenario(out error);
        }

        public bool TryResetScenario(string reason, out string error)
            => TryResetScenario(tracker != null ? tracker.ScenarioNumber : 0, reason, out error);

        public bool TryResetScenario(int expectedScenario, string reason, out string error)
        {
            if (!CanResetScenario(expectedScenario, reason, out error)) return false;
            tracker.ResetScenarioFromCell(reason.Trim());
            scaraSequence.ResetScenarioFromCell(reason.Trim()); fanucSequence.ResetScenarioFromCell(reason.Trim());
            controlledSequence = null; hasControlledSequence = false;
            ClearAutomaticTransfer();
            return true;
        }

        internal bool CanIssueLegacyCommand(Component target, RobotWaypointSequence requester, out string error)
        {
            if (!Ready(out error)) return false;
            return CheckCommandAdmission(target, requester, out error);
        }

        public bool CanMoveManualTcp(Component robot, Vector3 robotLocalTarget, out string error)
        {
            if (!CanReadReady(out error) || !ClockReady(out error)) return false;
            if (robot != scara && robot != fanuc) { error = "현재 셀의 SCARA 또는 FANUC를 선택하세요."; return false; }
            if (!CheckCommandAdmission(robot, null, out error)) return false;
            if (!ReferenceEquals(scaraGripper.HeldItem, null) || !ReferenceEquals(fanucGripper.HeldItem, null))
            { error = "수동 TCP 목표는 두 집게의 실제 보유가 비어 있을 때만 허용합니다."; return false; }
            if (!tracker.TryCheckPhysicalState(out error)) return false;
            bool isScara = robot == scara;
            var otherGripper = isScara ? fanucGripper : scaraGripper;
            if (!otherGripper.IsStopped) { error = "다른 로봇의 이동을 정지한 뒤 수동 TCP 목표를 요청하세요."; return false; }
            var ownZone = isScara ? layout.ScaraZone : layout.FanucZone;
            if (!TryReadZone(ownZone, isScara ? layout.ScaraFootprint : layout.FanucFootprint, out var ownFrame, out var ownBounds) ||
                !TryReadZone(layout.HandoffZone, layout.HandoffFootprint, out var sharedFrame, out var sharedBounds))
            { error = "현재 셀의 작업/인계 구역 경계를 확인하세요."; return false; }
            if (!ownZone.gameObject.activeInHierarchy) { error = "선택 로봇의 작업구역이 비활성입니다."; return false; }
            var frame = isScara ? scara.RobotFrame : fanuc.RobotFrame;
            var world = frame.TransformPoint(robotLocalTarget);
            // Shared restrictions take precedence where the authored footprints overlap.
            if (InZone(sharedFrame, sharedBounds, world))
            {
                if (!layout.HandoffZone.gameObject.activeInHierarchy || tracker.HandoffProductId != null)
                { error = "공유 인계구역이 비활성 또는 상품 점유 중입니다."; return false; }
                var otherTcp = isScara ? fanuc.Tcp : scara.Tcp;
                if (InZone(sharedFrame, sharedBounds, otherTcp.position))
                { error = "공유 인계구역에 다른 로봇의 TCP가 있습니다."; return false; }
                foreach (var record in tracker.Snapshot())
                    if (tracker.TryGetItem(record.Id, out var item) && item.gameObject.activeInHierarchy)
                    {
                        if (item.Body == null) { error = "등록 상품 " + record.Id + "의 실제 본체를 확인하세요."; return false; }
                        if (InZone(sharedFrame, sharedBounds, item.Body.position))
                        { error = "공유 인계구역에 등록 상품 " + record.Id + "의 실물이 있습니다."; return false; }
                    }
                return true;
            }
            if (!InZone(ownFrame, ownBounds, world)) { error = "수동 목표는 자기 작업구역 또는 비어 있는 공유 인계구역 안이어야 합니다."; return false; }
            return true;
        }

        private bool TryReadZone(Transform zone, MeshFilter footprint, out Transform frame, out Bounds bounds)
        {
            frame = null; bounds = default;
            if (zone == null || !zone.IsChildOf(transform)) return false;
            if (footprint == null || !footprint.transform.IsChildOf(zone) || footprint.sharedMesh == null) return false;
            frame = footprint.transform; bounds = footprint.sharedMesh.bounds;
            var scale = frame.lossyScale;
            return bounds.extents.x > 0 && bounds.extents.z > 0 && Mathf.Abs(scale.x) > .000001f &&
                Mathf.Abs(scale.y) > .000001f && Mathf.Abs(scale.z) > .000001f;
        }
        private static bool InZone(Transform frame, Bounds bounds, Vector3 world)
        {
            var local = frame.InverseTransformPoint(world);
            return local.x >= bounds.min.x && local.x <= bounds.max.x && local.z >= bounds.min.z && local.z <= bounds.max.z;
        }

        // Side-effect-free UI queries. Expensive path/IK/alignment validation remains at
        // execution time; reading a panel must never fault, stop or command the cell.
        public bool CanExecute(CellRunCommand command, out string error)
        {
            if (!Application.isPlaying) { error = "Play 상태에서 셀 명령을 요청하세요."; return false; }
            if (command == CellRunCommand.Stop) return policy.CanExecute(command, out error);
            if (command == CellRunCommand.Reset) return CanRecover(out error);
            if (!CanReadReady(out error) || !ClockReady(out error) || !policy.CanExecute(command, out error)) return false;
            if (command == CellRunCommand.Start && AutomaticPhase == AutomaticTransferPhase.Completed)
            { error = "자동 운전이 완료되었습니다. 권한을 반환하고 다음 상품을 준비하세요."; return false; }
            if (command == CellRunCommand.Start || command == CellRunCommand.Resume)
            {
                if (!AtRest) { error = "두 로봇을 정지한 뒤 셀을 시작/재개하세요."; return false; }
                if (!hasControlledSequence && (HasPendingSequence(scaraSequence) || HasPendingSequence(fanucSequence)))
                { error = "개별 중단 경로를 공통 제어에 인계하거나 기존 시험을 마치세요."; return false; }
            }
            return true;
        }

        public bool CanTakeControl(RobotWaypointSequence sequence, out string error)
        {
            if (IsFaulted)
            {
                if (!CanPrepareRecovery(out error)) return false;
                if (hasControlledSequence) { error = "이미 복구/운전 경로가 인계되어 있습니다. 대상을 교체하지 않습니다."; return false; }
                if (sequence == null || sequence != scaraSequence && sequence != fanucSequence)
                { error = "이 셀의 고장 경로만 복구 대상으로 인계할 수 있습니다."; return false; }
                var otherFaultPath = sequence == scaraSequence ? fanucSequence : scaraSequence;
                if (HasPendingSequence(otherFaultPath))
                { error = "다른 로봇에도 미완료 경로가 있습니다. 여러 중단 경로의 복구는 별도 지원이 필요합니다."; return false; }
                return sequence.CanPrepareFaultRecovery(this, out error);
            }
            if (!CanReadReady(out error) || !ClockReady(out error) || !policy.CanIssueRobotCommand(CellRobotCommandSource.Manual, out error)) return false;
            if (hasControlledSequence) { error = "이미 공통 제어 경로가 있습니다. 정지 후 권한을 반환하세요."; return false; }
            if (sequence == null || sequence != scaraSequence && sequence != fanucSequence)
            { error = "이 셀의 로봇 경로만 인계할 수 있습니다."; return false; }
            var other = sequence == scaraSequence ? fanucSequence : scaraSequence;
            if (!AtRest || sequence.State != RobotWaypointSequence.RunState.Paused || HasPendingSequence(other))
            { error = "다른 로봇의 시험을 마치고 인계할 경로 하나만 일시정지하세요."; return false; }
            return true;
        }

        public bool CanReleaseControl(out string error)
        {
            if (!CanReadReady(out error) || !policy.CanIssueRobotCommand(CellRobotCommandSource.Manual, out error)) return false;
            if (!hasControlledSequence) { error = "반환할 공통 제어 경로가 없습니다."; return false; }
            if (!AtRest) { error = "두 로봇을 정지한 뒤 권한을 반환하세요."; return false; }
            return true;
        }

        public bool CanIssueManualCommand(Component target, out string error) => CheckCommandAdmission(target, null, out error);

        private bool CheckCommandAdmission(Component target, RobotWaypointSequence requester, out string error)
        {
            if (!CanReadReady(out error)) return false;
            if (target == null) { error = "로봇 명령 대상을 확인하세요."; return false; }
            if (requester != null && ((requester != scaraSequence && requester != fanucSequence) || !target.transform.IsChildOf(requester.transform)))
            { error = "이 로봇의 순차 운전 요청이 아닙니다."; return false; }
            if (hasControlledSequence)
            {
                if (requester == null || requester != controlledSequence)
                { error = "공통 제어에 인계된 경로입니다. 공통 조작을 사용하거나 정지 후 권한을 반환하세요."; return false; }
                return policy.CanIssueRobotCommand(CellRobotCommandSource.Automatic, out error);
            }
            if (!policy.CanIssueRobotCommand(CellRobotCommandSource.Manual, out error)) return false;
            if ((scaraSequence.IsBusy && requester != scaraSequence) || (fanucSequence.IsBusy && requester != fanucSequence))
            { error = "셀의 개별 순차 시험 중입니다. 정지 후 수동/다른 자동 명령을 요청하세요."; return false; }
            if (target is RobotWaypointSequence && requester == null && !AtRest)
            { error = "두 로봇의 이동을 정지한 뒤 순차 시험을 시작/재개하세요."; return false; }
            return true;
        }

        private bool Ready(out string error)
        {
            SynchronizeFault();
            return CanReadReady(out error);
        }

        private bool CanReadReady(out string error)
        {
            if (IsFaulted) { error = FaultReason; return false; }
            if (!Application.isPlaying || !isActiveAndEnabled) { error = "활성 셀과 Play 상태를 확인하세요."; return false; }
            if (!Wired || hasControlledSequence && controlledSequence == null)
            { error = "셀의 두 로봇/상품 추적/인계 경로 연결을 확인하세요."; return false; }
            if (tracker.IsFaulted) { error = tracker.FaultReason; return false; }
            if (scaraSequence.State == RobotWaypointSequence.RunState.Faulted) { error = scaraSequence.LastError; return false; }
            if (fanucSequence.State == RobotWaypointSequence.RunState.Faulted) { error = fanucSequence.LastError; return false; }
            error = null;
            return true;
        }

        private static bool ClockReady(out string error)
        {
            error = null;
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPaused) { error = "에디터 일시정지를 해제하세요."; return false; }
#endif
            if (Time.timeScale <= 0f) { error = "시간 배율을 복구하세요."; return false; }
            return true;
        }

        // Shared readiness for acknowledging or explicitly adopting a stopped fault path.
        // Preparing an adoption never clears a fault or dispatches a robot command.
        private bool CanPrepareRecovery(out string error)
        {
            if (!policy.CanExecute(CellRunCommand.Reset, out error) || !ClockReady(out error)) return false;
            if (!isActiveAndEnabled || !Wired || hasControlledSequence && controlledSequence == null)
            { error = "셀 관리자·두 로봇·인계 경로·상품 추적 연결을 복원하세요."; return false; }
            if (!AtRest) { error = "두 로봇과 순차 운전을 정지한 뒤 고장을 해제하세요."; return false; }
            if (!tracker.TryCheckPhysicalState(out error)) return false;
            foreach (var gripper in new[] { scaraGripper, fanucGripper })
                if (gripper.HeldItem != null && (!gripper.HeldItem.IsReady || gripper.HeldItem.Owner != gripper))
                { error = "그리퍼의 실제 보유를 확인할 수 없습니다."; return false; }
            return true;
        }

        internal bool CanConfirmProductLoss(CellProductTracker requester, out string error)
        {
            error = null;
            if (!Application.isPlaying || !isActiveAndEnabled || !IsFaulted || !Wired || requester != tracker)
            { error = "같은 활성 셀의 Faulted 상태에서만 상품을 명시 정리합니다."; return false; }
            if (!ClockReady(out error)) return false;
            if (!AtRest || hasControlledSequence || HasPendingSequence(scaraSequence) || HasPendingSequence(fanucSequence))
            { error = "미완료/인계 경로가 있거나 로봇이 움직입니다. 저장된 상품 경로를 상품 정리로 무효화하지 않습니다."; return false; }
            if (!ReferenceEquals(scaraGripper.HeldItem, null) || !ReferenceEquals(fanucGripper.HeldItem, null))
            { error = "두 집게의 실제 보유를 비운 상태에서만 상품을 명시 정리합니다."; return false; }
            return true;
        }

        private bool CanRecover(out string error)
        {
            if (!CanPrepareRecovery(out error)) return false;
            foreach (var sequence in new[] { scaraSequence, fanucSequence })
                if (sequence.State == RobotWaypointSequence.RunState.Faulted && !sequence.CanAcknowledgeFault(this, out error)) return false;
            return true;
        }

        private bool ValidateRecovery(out string error)
        {
            if (!CanRecover(out error)) return false;
            return ValidateRecoveryPrograms(out error);
        }

        private bool ValidateRecoveryPrograms(out string error)
        {
            if (!scara.TryReadFkDiagnostics(out var sd, out error) || !fanuc.TryReadFkDiagnostics(out var fd, out error)) return false;
            if (sd.PositionErrorMetres > .00001f || sd.YawErrorDegrees > .005f || fd.PositionErrorMetres > .00001f || fd.RotationErrorDegrees > .005f)
            { error = "실제 TCP와 FK가 일치하지 않습니다. 고장을 유지합니다."; return false; }
            foreach (var sequence in new[] { scaraSequence, fanucSequence })
                if ((sequence.State == RobotWaypointSequence.RunState.Paused || sequence.State == RobotWaypointSequence.RunState.Faulted) &&
                    !sequence.TryValidateResume(out error)) return false;
            return true;
        }

        private bool Wired => layout != null && tracker != null && tracker.isActiveAndEnabled &&
            scara != null && fanuc != null && layout.Scara == scara && layout.Fanuc == fanuc &&
            scara.transform.IsChildOf(transform) && fanuc.transform.IsChildOf(transform) &&
            scara.IsReady && fanuc.IsReady && scaraGripper != null && fanucGripper != null &&
            scaraGripper.IsReady && fanucGripper.IsReady && scaraSequence != null && fanucSequence != null &&
            scaraSequence.IsReady && fanucSequence.IsReady;
        private bool AtRest => scaraGripper != null && fanucGripper != null && scaraGripper.IsStopped && fanucGripper.IsStopped;
        private static bool HasPendingSequence(RobotWaypointSequence sequence) => sequence.HasPendingProgram;

        private void FixedUpdate()
        {
            if (!IsAutomaticContinuous || AutomaticPhase != AutomaticTransferPhase.WaitingForInput || State != CellRunState.Running || !ClockReady(out _)) return;
            SynchronizeFault();
            if (IsFaulted) return;
            if (!tracker.TryCheckPhysicalState(out var error, out var causeProductId)) { Fault(error, FaultSource.ProductTracker, causeProductId); return; }
            if (tracker.InputProductId != null) return; // A retained Input record is retried by Update after explicit recovery.
            automaticInputPreparation = true;
            try
            {
                if (!tracker.TrySupplyNextFromCell(this, Time.fixedDeltaTime, out error)) Fault(error);
            }
            finally { automaticInputPreparation = false; }
        }

        private void Update()
        {
            SynchronizeFault();
            UpdateProgress();
            if (State != CellRunState.Running || !hasControlledSequence || !ClockReady(out _)) return;
            if (AutomaticPhase == AutomaticTransferPhase.WaitingForInput)
            {
                if (!tracker.TryCheckPhysicalState(out var error, out var causeProductId)) { Fault(error, FaultSource.ProductTracker, causeProductId); return; }
                if (tracker.InputProductId == null)
                {
                    var waiting = tracker.WaitingSnapshot();
                    if (waiting.Count == 0 || waiting[0] != AutomaticProductId)
                    { Fault("대기하던 선두 상품이 바뀌었습니다. 자동 대상을 교체하지 않습니다."); return; }
                    if (!tracker.IsNextWaitingAligned()) { CheckSupplyProgress(); return; }
                }
                ClearSupplyProgress();
                bool prepared;
                automaticInputPreparation = true;
                try
                {
                    if (tracker.InputProductId == null && !tracker.TryFeedNextFromCell(this, out _, out error)) { Fault(error); return; }
                    if (tracker.InputProductId != AutomaticProductId || !tracker.TryGet(AutomaticProductId, out var record) || record.Destination != automaticDestination ||
                        record.Location != ProductLocation.Input || !tracker.TryGetItem(AutomaticProductId, out var item))
                    { Fault("연속 운전의 투입 상품·원래 실물·목적지를 확인하세요."); return; }
                    prepared = scaraSequence.TryPrepareScaraTransfer(this, layout, scaraGripper, item, out error);
                }
                finally { automaticInputPreparation = false; }
                if (!prepared) { Fault(error); return; }
                controlledSequence = scaraSequence;
                AutomaticPhase = AutomaticTransferPhase.Scara;
                ResumeControlledPath(out _);
            }
            else if (AutomaticPhase == AutomaticTransferPhase.Scara && controlledSequence == scaraSequence && scaraSequence.State == RobotWaypointSequence.RunState.Completed)
            {
                if (!tracker.TryGet(AutomaticProductId, out var record) || record.Location != ProductLocation.Handoff || record.Destination != automaticDestination ||
                    !tracker.TryGetItem(AutomaticProductId, out var item) || item != scaraSequence.CycleItem ||
                    !AtRest || scaraGripper.HeldItem != null || fanucGripper.HeldItem != null)
                { Fault("자동 인계 상품·원래 실물·목적지·두 집게의 정지/보유를 확인하세요."); return; }
                if (!tracker.TryCheckPhysicalState(out var error, out var causeProductId)) { Fault(error, FaultSource.ProductTracker, causeProductId); return; }
                if (!TryCheckScaraRetreat(out error)) { Fault(error); return; }
                // Preparation is atomic. Keep the SCARA permission if FANUC validation fails.
                bool prepared;
                automaticHandoffPreparation = true;
                try { prepared = fanucSequence.TryPrepareFanucTransfer(this, layout, fanucGripper, item, automaticDestination, out error); }
                finally { automaticHandoffPreparation = false; }
                if (!prepared) { Fault(error); return; }
                controlledSequence = fanucSequence;
                AutomaticPhase = AutomaticTransferPhase.Fanuc;
                ResumeControlledPath(out _);
            }
            else if (AutomaticPhase == AutomaticTransferPhase.Fanuc && controlledSequence == fanucSequence && fanucSequence.State == RobotWaypointSequence.RunState.Completed)
            {
                var output = automaticDestination == ProductDestination.A ? ProductLocation.OutputA : ProductLocation.OutputB;
                if (!tracker.TryGet(AutomaticProductId, out var record) || record.Location != output || record.Destination != automaticDestination ||
                    !tracker.TryGetItem(AutomaticProductId, out var item) || item != fanucSequence.CycleItem || item.Owner != null ||
                    !AtRest || scaraGripper.HeldItem != null || fanucGripper.HeldItem != null || fanucSequence.GripCount != 1 || fanucSequence.ReleaseCount != 1)
                { Fault("자동 단품의 배출 기록·실제 보유·행동 횟수가 맞지 않습니다."); return; }
                var waiting = tracker.WaitingSnapshot();
                if (IsAutomaticContinuous && waiting.Count > 0)
                {
                    if (!tracker.TryGet(waiting[0], out var next) || next.Location != ProductLocation.Waiting)
                    { Fault("다음 선두 상품의 대기 기록이 유효하지 않습니다."); return; }
                    AutomaticProductId = next.Id; automaticDestination = next.Destination;
                    AutomaticPhase = AutomaticTransferPhase.WaitingForInput;
                }
                else
                {
                    AutomaticPhase = AutomaticTransferPhase.Completed;
                    TryExecute(CellRunCommand.Stop, out _);
                }
            }
        }

        internal bool AllowsAutomaticHandoffPreparation(RobotWaypointSequence candidate) => automaticHandoffPreparation &&
            AutomaticPhase == AutomaticTransferPhase.Scara && State == CellRunState.Running &&
            hasControlledSequence && controlledSequence == scaraSequence && scaraSequence.State == RobotWaypointSequence.RunState.Completed &&
            candidate == fanucSequence && AtRest;

        internal bool AllowsAutomaticInputPreparation(Component candidate) => automaticInputPreparation && IsAutomaticContinuous &&
            AutomaticPhase == AutomaticTransferPhase.WaitingForInput && State == CellRunState.Running &&
            hasControlledSequence && controlledSequence == fanucSequence && fanucSequence.State == RobotWaypointSequence.RunState.Completed &&
            (candidate == tracker || candidate == scaraSequence) && !HasPendingSequence(scaraSequence) && AtRest &&
            ReferenceEquals(scaraGripper.HeldItem, null) && ReferenceEquals(fanucGripper.HeldItem, null);

        private void ClearAutomaticTransfer()
        {
            AutomaticPhase = AutomaticTransferPhase.None; AutomaticProductId = null; IsAutomaticContinuous = false;
            ClearProgress();
        }
        private void ClearProgress()
        {
            ClearSupplyProgress();
            ClearRobotProgress();
            ProgressKind = AutomaticProgressKind.None; ProgressActiveSeconds = 0;
            ProgressProductId = null; ProgressEquipmentId = null; ProgressStep = null;
            observedPhase = AutomaticTransferPhase.None; observedProgramRevision = 0; observedStepIndex = -1;
        }
        private void UpdateProgress()
        {
            if (AutomaticPhase == AutomaticTransferPhase.None || AutomaticPhase == AutomaticTransferPhase.Completed)
            { ClearProgress(); return; }
            if (State != CellRunState.Running || !hasControlledSequence || !ClockReady(out _)) return;
            bool supplying = AutomaticPhase == AutomaticTransferPhase.WaitingForInput;
            var sequence = AutomaticPhase == AutomaticTransferPhase.Scara ? scaraSequence : fanucSequence;
            uint revision = supplying ? 0 : sequence.ProgramRevision;
            int step = supplying ? -1 : sequence.StepIndex;
            if (observedPhase != AutomaticPhase || ProgressProductId != AutomaticProductId || observedProgramRevision != revision || observedStepIndex != step)
            {
                ClearSupplyProgress();
                ClearRobotProgress();
                ProgressActiveSeconds = 0;
                observedPhase = AutomaticPhase; observedProgramRevision = revision; observedStepIndex = step;
            }
            ProgressKind = supplying ? AutomaticProgressKind.NormalInputSupply : AutomaticProgressKind.RobotStep;
            ProgressProductId = AutomaticProductId;
            ProgressEquipmentId = supplying ? null : AutomaticPhase == AutomaticTransferPhase.Scara ? layout.ScaraId : layout.FanucId;
            ProgressStep = supplying ? "선두 상품 공급 중" : sequence.StepName;
            // Scaled simulation seconds, matching motion; distinct from time without actual supply progress.
            ProgressActiveSeconds += Time.deltaTime;
            if (!supplying) CheckRobotProgress(sequence);
        }
        private void ClearRobotProgress()
        {
            observedRobotCommand = 0; observedAppliedSeconds = 0; RobotNoProgressSeconds = 0;
        }
        private void CheckRobotProgress(RobotWaypointSequence sequence)
        {
            bool isScara = sequence == scaraSequence;
            if (sequence.State != RobotWaypointSequence.RunState.Running || !(isScara ? scara.IsTrajectoryMoving : fanuc.IsTcpMoving)) return;
            uint command = isScara ? scara.CommandVersion : fanuc.CommandVersion;
            double applied = isScara ? scara.AppliedMotionSeconds : fanuc.AppliedMotionSeconds;
            if (observedRobotCommand != command)
            {
                observedRobotCommand = command; observedAppliedSeconds = 0;
                // A resume replans from the current pose. Preserve the stalled time until it actually advances.
            }
            if (applied > observedAppliedSeconds)
            {
                observedAppliedSeconds = applied; RobotNoProgressSeconds = 0;
                return;
            }
            RobotNoProgressSeconds += Time.deltaTime;
            if (RobotNoProgressSeconds >= RobotNoProgressTimeoutSeconds)
                Fault(ProgressEquipmentId + "의 " + sequence.StepName + " 이동이 " + RobotNoProgressSeconds.ToString("F2")
                    + "초 동안 진행되지 않았습니다. 실제 로봇 구동을 확인하세요.", isScara ? FaultSource.Scara : FaultSource.Fanuc);
        }
        private void ClearSupplyProgress()
        {
            supplySampled = false; bestSupplyDistance = 0; bestSupplyAngle = 0; SupplyNoProgressSeconds = 0;
        }
        private void CheckSupplyProgress()
        {
            if (!tracker.TryGetItem(AutomaticProductId, out var item)) return; // Physical validation reports missing originals first.
            float distance = Vector3.Distance(item.Body.position, layout.InputStop.position);
            float angle = RobotGripper.RotationError(item.Body.rotation, layout.InputStop.rotation);
            if (!supplySampled)
            {
                supplySampled = true; bestSupplyDistance = distance; bestSupplyAngle = angle;
                return;
            }
            // Keep the best evidence so back-and-forth jitter cannot repeatedly restart the clock.
            if (distance <= bestSupplyDistance - .0001f || angle <= bestSupplyAngle - .05f)
            {
                bestSupplyDistance = Mathf.Min(bestSupplyDistance, distance); bestSupplyAngle = Mathf.Min(bestSupplyAngle, angle);
                SupplyNoProgressSeconds = 0;
                return;
            }
            SupplyNoProgressSeconds += Time.deltaTime;
            if (SupplyNoProgressSeconds >= SupplyNoProgressTimeoutSeconds)
                Fault("선두 상품 " + AutomaticProductId + "의 공급이 " + SupplyNoProgressSeconds.ToString("F2")
                    + "초 동안 진행되지 않았습니다. 실제 공급/정렬을 확인하세요.", FaultSource.InputSupply);
        }
        private void SynchronizeFault()
        {
            if (!Application.isPlaying || IsFaulted) return;
            if (tracker != null && tracker.IsFaulted) { Fault(tracker.FaultReason, FaultSource.ProductTracker, tracker.FaultProductId); return; }
            if (!isActiveAndEnabled || !Wired) { Fault("셀 운전 관리자 또는 두 로봇/상품 추적 연결이 유효하지 않습니다.", WiringFaultSource()); return; }
            if (hasControlledSequence && controlledSequence == null) { Fault("공통 제어에 인계된 로봇 경로가 사라졌습니다."); return; }
            if (scaraSequence.State == RobotWaypointSequence.RunState.Faulted) Fault(scaraSequence.LastError, FaultSource.Scara);
            else if (fanucSequence.State == RobotWaypointSequence.RunState.Faulted) Fault(fanucSequence.LastError, FaultSource.Fanuc);
            else if (!resumeDispatchInProgress && hasControlledSequence && State == CellRunState.Running && controlledSequence.State == RobotWaypointSequence.RunState.Paused)
            {
                // Individual Stop/Pause remains available. Reflect its interruption at cell
                // level too; never silently re-dispatch on the next Update.
                policy.TryExecute(CellRunCommand.Pause, out _);
                StopBoth();
            }
        }
        private void OnTrackerFault(string reason) => Fault(reason, FaultSource.ProductTracker, tracker.FaultProductId);
        private FaultSource WiringFaultSource()
        {
            if (!isActiveAndEnabled || layout == null) return FaultSource.Cell;
            if (tracker == null || !tracker.isActiveAndEnabled) return FaultSource.ProductTracker;
            bool scaraReady = scara != null && layout.Scara == scara && scara.transform.IsChildOf(transform) && scara.IsReady &&
                scaraGripper != null && scaraGripper.IsReady && scaraSequence != null && scaraSequence.IsReady;
            bool fanucReady = fanuc != null && layout.Fanuc == fanuc && fanuc.transform.IsChildOf(transform) && fanuc.IsReady &&
                fanucGripper != null && fanucGripper.IsReady && fanucSequence != null && fanucSequence.IsReady;
            return !scaraReady && fanucReady ? FaultSource.Scara : scaraReady && !fanucReady ? FaultSource.Fanuc : FaultSource.Cell;
        }
        private void Fault(string reason, FaultSource source = FaultSource.Cell, string causeProductId = null)
        {
            if (!policy.TryFault(string.IsNullOrWhiteSpace(reason) ? "셀 운전 오류가 발생했습니다." : reason, out _)) return;
            var sequence = source == FaultSource.Scara ? scaraSequence : source == FaultSource.Fanuc ? fanucSequence : null;
            string equipmentId = source == FaultSource.Scara ? layout.ScaraId : source == FaultSource.Fanuc ? layout.FanucId : null;
            ActiveFault = new FaultRecord(source, equipmentId, sequence != null ? sequence.CycleProductId : AutomaticProductId,
                source == FaultSource.InputSupply ? AutomaticProductId : causeProductId,
                sequence != null ? sequence.State == RobotWaypointSequence.RunState.Idle ? "대기" : sequence.StepName
                    : source == FaultSource.InputSupply ? "선두 상품 공급 중" : AutomaticPhase.ToString(), policy.FaultReason);
            StopBoth();
        }
        private void StopBoth()
        {
            if (scaraSequence != null) scaraSequence.Pause();
            if (fanucSequence != null) fanucSequence.Pause();
            if (scara != null) scara.StopAll();
            if (fanuc != null) fanuc.StopAll();
        }
        private void OnDisable() { if (Application.isPlaying) Fault("셀 운전 관리자가 비활성화되었습니다."); }
        private void OnDestroy() { if (tracker != null) tracker.Faulted -= OnTrackerFault; }
    }

    // Cache cell membership, including a missing controller. Removing/disabling the manager
    // must never turn a managed robot back into an unguarded RobotLab robot.
    internal readonly struct CellCommandGate
    {
        private readonly bool managed;
        private readonly Component target;
        private readonly CellRunController controller;
        internal CellCommandGate(Component target)
        {
            this.target = target;
            var layout = target.GetComponentInParent<LineWatchCellLayout>(true);
            managed = layout != null;
            controller = managed ? layout.GetComponent<CellRunController>() : null;
        }
        internal bool CanIssue(RobotWaypointSequence requester, out string error)
        {
            error = null;
            if (!managed) return true;
            if (controller == null) { error = "셀 운전 관리자가 없습니다. 로봇 명령을 차단합니다."; return false; }
            return controller.CanIssueLegacyCommand(target, requester, out error);
        }
        internal bool CanIssueTcp(RobotWaypointSequence requester, Vector3 localTarget, out string error)
        {
            if (!CanIssue(requester, out error)) return false;
            return !managed || requester != null || controller.CanMoveManualTcp(target, localTarget, out error);
        }
        internal bool IsManagedBy(CellRunController owner) => managed && controller != null && controller == owner;
    }
}
