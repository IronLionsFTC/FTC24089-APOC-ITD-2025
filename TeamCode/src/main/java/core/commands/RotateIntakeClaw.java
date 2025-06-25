package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import core.parameters.PositionalBounds;
import core.state.Subsystems;
import core.subsystems.Intake;

public class RotateIntakeClaw extends CommandBase {

    private Intake intakeSubsystem;
    private DoubleSupplier right;
    private DoubleSupplier left;

    public RotateIntakeClaw(Intake intakeSubsystem, DoubleSupplier right, DoubleSupplier left) {
        this.intakeSubsystem = intakeSubsystem;
        this.right = right;
        this.left = left;
    }

    @Override
    // Run forever, rotating the claw by the left and right inputs
    public void execute() {
        if (intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawDown || intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing) {
            this.intakeSubsystem.rotateIntakeClaw((right.getAsDouble() - left.getAsDouble()) / 2); // Scale speed down
        } else {
            this.intakeSubsystem.setIntakeClawRotation(PositionalBounds.ServoPositions.ClawPositions.yawRest);
        }
    }
}
