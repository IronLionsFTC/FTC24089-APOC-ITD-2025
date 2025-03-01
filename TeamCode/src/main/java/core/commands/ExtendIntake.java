package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Intake;

public class ExtendIntake extends CommandBase {

    private final Intake intakeSubsystem;
    private final double clawRotation;

    public ExtendIntake(Intake intakeSubsystem, Double clawRotation) {
        this.intakeSubsystem = intakeSubsystem;

        if (clawRotation.isNaN() || clawRotation == null) {
            this.clawRotation = 0;
        } else {
            this.clawRotation = clawRotation;
        }
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
    public void execute() {
        // If the intake has begun moving, and is nearly at full extension, fold down the claw
        if (intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawUp && intakeSubsystem.isSlidesPartiallyExtended()) {
            intakeSubsystem.nextState();
        }
    }

    @Override
    public boolean isFinished() {
        // If the subsystem is not in the correct states, immediately exit
        if (intakeSubsystem.state != Subsystems.IntakeState.ExtendedClawUp && intakeSubsystem.state != Subsystems.IntakeState.ExtendedClawDown) {
            return true;
        }
        // If the slides are 70% extended, finish command
        return intakeSubsystem.isSlidesExtended() && intakeSubsystem.gimblePitchDone();
    }
}
