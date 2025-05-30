package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Intake;

public class WaitAndGrabSample extends CommandBase {
    private Intake intakeSubsystem;

    public WaitAndGrabSample(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        if (this.intakeSubsystem.isClawHoveringOverSample()) {
            this.intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawGrabbing;
        } else {
            this.intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawDown;
        }
    }

    @Override
    public boolean isFinished() {
        return this.intakeSubsystem.hasClawClosed();
    }
}
