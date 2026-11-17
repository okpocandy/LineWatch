using UnityEngine;

namespace LineWatch
{
    // Owns command permission for one explicitly adopted, already-paused robot program.
    // Product dispatch and the two-robot picking cycle remain separate from this controller.
    [DefaultExecutionOrder(300)]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(LineWatchCellLayout))]
    public sealed class CellRunController : MonoBehaviour
    {
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

        public CellRunState State => policy.State;
        public string FaultReason => policy.FaultReason;
        public string LastAcknowledgedFaultReason => policy.LastAcknowledgedFaultReason;
        public bool IsFaulted => policy.IsFaulted;
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
                return policy.TryExecute(command, out error);
            }
            bool resumePath = false;
            if ((command == CellRunCommand.Start || command == CellRunCommand.Resume) && hasControlledSequence && controlledSequence.State != RobotWaypointSequence.RunState.Completed)
            {
                if (!controlledSequence.TryValidateResume(out error)) return false;
                resumePath = true;
            }
            if (!policy.TryExecute(command, out error)) return false;
            if (command == CellRunCommand.Pause) StopBoth();
            if (resumePath)
            {
                // The gate re-reads cell health during this synchronous dispatch. Do not
                // mistake the still-paused robot for an external stop before it commits.
                resumeDispatchInProgress = true;
                try
                {
                    if (!controlledSequence.TryResumeFromCell(this, out error))
                    {
                        Fault(error);
                        return false;
                    }
                }
                finally { resumeDispatchInProgress = false; }
            }
            return true;
        }

        public bool TryTakeControl(RobotWaypointSequence sequence, out string error)
        {
            if (!Ready(out error) || !CanTakeControl(sequence, out error)) return false;
            if (!sequence.TryValidateResume(out error)) return false;
            controlledSequence = sequence;
            hasControlledSequence = true;
            return true;
        }

        public bool TryReleaseControl(out string error)
        {
            if (!Ready(out error) || !CanReleaseControl(out error)) return false;
            controlledSequence = null;
            hasControlledSequence = false;
            return true;
        }

        internal bool CanIssueLegacyCommand(Component target, RobotWaypointSequence requester, out string error)
        {
            if (!Ready(out error)) return false;
            return CheckCommandAdmission(target, requester, out error);
        }

        // Side-effect-free UI queries. Expensive path/IK/alignment validation remains at
        // execution time; reading a panel must never fault, stop or command the cell.
        public bool CanExecute(CellRunCommand command, out string error)
        {
            if (!Application.isPlaying) { error = "Play 상태에서 셀 명령을 요청하세요."; return false; }
            if (command == CellRunCommand.Stop) return policy.CanExecute(command, out error);
            if (command == CellRunCommand.Reset) return CanRecover(out error);
            if (!CanReadReady(out error) || !ClockReady(out error) || !policy.CanExecute(command, out error)) return false;
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

        private bool CanRecover(out string error)
        {
            if (!policy.CanExecute(CellRunCommand.Reset, out error) || !ClockReady(out error)) return false;
            if (!isActiveAndEnabled || !Wired || hasControlledSequence && controlledSequence == null)
            { error = "셀 관리자·두 로봇·인계 경로·상품 추적 연결을 복원하세요."; return false; }
            if (!AtRest) { error = "두 로봇과 순차 운전을 정지한 뒤 고장을 해제하세요."; return false; }
            foreach (var sequence in new[] { scaraSequence, fanucSequence })
                if (sequence.State == RobotWaypointSequence.RunState.Faulted && !sequence.CanAcknowledgeFault(this, out error)) return false;
            if (!tracker.TryCheckPhysicalState(out error)) return false;
            foreach (var gripper in new[] { scaraGripper, fanucGripper })
                if (gripper.HeldItem != null && (!gripper.HeldItem.IsReady || gripper.HeldItem.Owner != gripper))
                { error = "그리퍼의 실제 보유를 확인할 수 없습니다."; return false; }
            return true;
        }

        private bool ValidateRecovery(out string error)
        {
            if (!CanRecover(out error)) return false;
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
        private static bool HasPendingSequence(RobotWaypointSequence sequence) => sequence.State != RobotWaypointSequence.RunState.Idle &&
            sequence.State != RobotWaypointSequence.RunState.Completed;

        private void Update() => SynchronizeFault();
        private void SynchronizeFault()
        {
            if (!Application.isPlaying || IsFaulted) return;
            if (tracker != null && tracker.IsFaulted) { Fault(tracker.FaultReason); return; }
            if (!isActiveAndEnabled || !Wired) { Fault("셀 운전 관리자 또는 두 로봇/상품 추적 연결이 유효하지 않습니다."); return; }
            if (hasControlledSequence && controlledSequence == null) { Fault("공통 제어에 인계된 로봇 경로가 사라졌습니다."); return; }
            if (scaraSequence.State == RobotWaypointSequence.RunState.Faulted) Fault(scaraSequence.LastError);
            else if (fanucSequence.State == RobotWaypointSequence.RunState.Faulted) Fault(fanucSequence.LastError);
            else if (!resumeDispatchInProgress && hasControlledSequence && State == CellRunState.Running && controlledSequence.State == RobotWaypointSequence.RunState.Paused)
            {
                // Individual Stop/Pause remains available. Reflect its interruption at cell
                // level too; never silently re-dispatch on the next Update.
                policy.TryExecute(CellRunCommand.Pause, out _);
                StopBoth();
            }
        }
        private void OnTrackerFault(string reason) => Fault(reason);
        private void Fault(string reason)
        {
            if (!policy.TryFault(string.IsNullOrWhiteSpace(reason) ? "셀 운전 오류가 발생했습니다." : reason, out _)) return;
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
        internal bool IsManagedBy(CellRunController owner) => managed && controller != null && controller == owner;
    }
}
