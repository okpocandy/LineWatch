namespace LineWatch
{
    public enum CellRunState { Stopped, Running, Paused, Faulted }
    public enum CellRunCommand { Start, Pause, Resume, Stop, Reset }
    public enum CellRobotCommandSource { Manual, Automatic }

    // Pure cell policy. The scene coordinator must check readiness, stop both robots and
    // enforce command admission at the actual entry points before using this model live.
    // RobotWaypointSequence owns robot steps; CellProductTracker/RobotGripper own products.
    public sealed class CellRunStateMachine
    {
        public CellRunState State { get; private set; } = CellRunState.Stopped;
        public string FaultReason { get; private set; }
        public string LastAcknowledgedFaultReason { get; private set; }
        public bool IsFaulted => State == CellRunState.Faulted;

        public bool TryExecute(CellRunCommand command, out string error)
        {
            if (!CanExecute(command, out error)) return false;
            switch (command)
            {
                case CellRunCommand.Start:
                case CellRunCommand.Resume: State = CellRunState.Running; break;
                case CellRunCommand.Pause: State = CellRunState.Paused; break;
                case CellRunCommand.Stop: if (!IsFaulted) State = CellRunState.Stopped; break;
                case CellRunCommand.Reset:
                    LastAcknowledgedFaultReason = FaultReason;
                    FaultReason = null;
                    State = CellRunState.Stopped;
                    break;
            }
            return true;
        }

        public bool CanExecute(CellRunCommand command, out string error)
        {
            error = null;
            switch (command)
            {
                case CellRunCommand.Start:
                    if (State != CellRunState.Stopped) return Reject("정지 상태에서만 시작할 수 있습니다.", out error);
                    return true;
                case CellRunCommand.Pause:
                    if (State != CellRunState.Running) return Reject("운전 중에만 일시정지할 수 있습니다.", out error);
                    return true;
                case CellRunCommand.Resume:
                    if (State != CellRunState.Paused) return Reject("일시정지 상태에서만 재개할 수 있습니다.", out error);
                    return true;
                case CellRunCommand.Stop:
                    // A stop request is repeatable, but never acknowledges or clears a fault.
                    return true;
                case CellRunCommand.Reset:
                    if (!IsFaulted) return Reject("고장 상태에서만 명시적으로 해제합니다. 상품/통계 초기화는 지원하지 않습니다.", out error);
                    return true;
                default:
                    return Reject("정의된 셀 운전 명령이 필요합니다.", out error);
            }
        }

        public bool TryFault(string reason, out string error)
        {
            if (string.IsNullOrWhiteSpace(reason)) return Reject("고장 원인이 필요합니다.", out error);
            if (IsFaulted) return Reject("이미 고장 상태입니다. 최초 원인을 유지합니다.", out error);
            FaultReason = reason;
            State = CellRunState.Faulted;
            error = null;
            return true;
        }

        // Read-only policy query, not a command dispatch or a lock. Pause/stop/fault commands
        // bypass this robot-motion policy so that stopping remains possible in every state.
        public bool CanIssueRobotCommand(CellRobotCommandSource source, out string error)
        {
            error = null;
            switch (source)
            {
                case CellRobotCommandSource.Manual:
                    if (State == CellRunState.Stopped) return true;
                    return Reject("수동 로봇 명령은 셀 정지 상태에서만 허용합니다.", out error);
                case CellRobotCommandSource.Automatic:
                    if (State == CellRunState.Running) return true;
                    return Reject("자동 로봇 명령은 셀 운전 상태에서만 허용합니다.", out error);
                default:
                    return Reject("정의된 로봇 명령 출처가 필요합니다.", out error);
            }
        }

        private static bool Reject(string reason, out string error) { error = reason; return false; }
    }
}
