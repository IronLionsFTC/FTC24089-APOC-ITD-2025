package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Intake;

public class ExtendIntake extends CommandBase {

    private final Intake intakeSubsystem;

    public ExtendIntake(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        // If transferring and need to extend, swap states
        if (intakeSubsystem.state == Subsystems.IntakeState.RetractedClawClosed) {
            intakeSubsystem.nextState();
        }

        // Once states have been swapped, or if it was already ready
        if (intakeSubsystem.state == Subsystems.IntakeState.RetractedClawOpen) {
            intakeSubsystem.nextState();
        }
    }

    @Override
    public boolean isFinished() {
        if (intakeSubsystem.state != Subsystems.IntakeState.ExtendedClawUp) {
            return true;
        }

        return intakeSubsystem
    }
}
