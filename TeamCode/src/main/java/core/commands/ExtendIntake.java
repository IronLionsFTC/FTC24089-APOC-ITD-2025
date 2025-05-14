package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.parameters.PositionalBounds;
import core.state.Subsystems;
import core.subsystems.Intake;

public class ExtendIntake extends CommandBase {

    private final Intake intakeSubsystem;
    private final double clawRotation;
    private final double extension;

    public ExtendIntake(Intake intakeSubsystem, Double clawRotation, Double extension) {
        this.intakeSubsystem = intakeSubsystem;

        if (extension.isNaN() || extension == null) {
            this.extension = PositionalBounds.SlidePositions.IntakePositions.extended;
        } else {
            this.extension = extension;
        }

        if (clawRotation.isNaN() || clawRotation == null) {
            this.clawRotation = 0.5;
        } else {
            this.clawRotation = clawRotation;
        }

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawUp;
    }

    @Override
    public void execute() {
        this.intakeSubsystem.setExtension(extension);
        // If the intake has begun moving, and is nearly at full extension, fold down the claw
        if (intakeSubsystem.isSlidesPartiallyExtended() && intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawUp) {
            intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawDown;
        }

        if (intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawDown) this.intakeSubsystem.setIntakeClawRotation(this.clawRotation);
    }

    @Override
    public boolean isFinished() {
        // If the slides are 70% extended, finish command
        return intakeSubsystem.isSlidesExtended();
    }
}
