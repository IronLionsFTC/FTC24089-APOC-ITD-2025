package core.state;

public class Subsystems {

    // Claw states
    public enum ClawState {
        StrongGripClosed,
        WeakGripClosed,
        Open,
        WideOpen
    }

    public enum IntakeState {
        RetractedClawOpen,
        ExtendedClawUp,
        ExtendedClawDown,
        ExtendedClawGrabbing,
        RetractedClawClosed
    }
}
