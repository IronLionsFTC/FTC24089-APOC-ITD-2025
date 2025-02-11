package core.commands;

import java.util.function.DoubleSupplier;

import core.subsystems.Drivebase;
import core.subsystems.Intake;

public class CMD {

    // Extends the intake, automatically folding down the claw and rotating to 0 or x degrees.
    public ExtendIntake extendIntake(Intake intakeSubsystem) { return new ExtendIntake(intakeSubsystem, (double)0); }
    public ExtendIntake extendIntake(Intake intakeSubsystem, double r) { return new ExtendIntake(intakeSubsystem, r); }

    // PERMANENTLY bind the drivebase subsystem to some double suppliers, usually joystick axis.
    public SetDriveVector setDriveVector(Drivebase drivebaseSubsystem, DoubleSupplier x, DoubleSupplier y) {
        return new SetDriveVector(drivebaseSubsystem, x, y);
    }

    // PERMANENTLY bind the intake subsystem's gimble yaw rotation to some double suppliers
    public RotateIntakeClaw rotateIntakeClaw(Intake intakeSubsystem, DoubleSupplier right, DoubleSupplier left) {
        return new RotateIntakeClaw(intakeSubsystem, right, left);
    }
}
