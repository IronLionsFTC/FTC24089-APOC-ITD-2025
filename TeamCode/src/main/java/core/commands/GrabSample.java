package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Intake;

public class GrabSample extends CommandBase {

    private Intake intakeSubsystem;

    public GrabSample(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawDown) {
            this.intakeSubsystem.nextState();
        }
    }

    @Override
    public boolean isFinished() {
        return this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing && this.intakeSubsystem.hasClawClosed();
    }
}
