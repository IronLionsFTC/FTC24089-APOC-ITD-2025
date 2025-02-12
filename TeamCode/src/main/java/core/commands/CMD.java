package core.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import core.subsystems.Drivebase;
import core.subsystems.Intake;

public class CMD {

    // Extensions of builtin commands
    public static Command sleep(long ms) { return new WaitCommand(ms); }
    public static Command sleepUntil(BooleanSupplier condition) { return new WaitUntilCommand(condition); }

    // Extends the intake, automatically folding down the claw and rotating to 0 or x degrees.
    public static ExtendIntake extendIntake(Intake intakeSubsystem) { return new ExtendIntake(intakeSubsystem, (double)0); }
    public static ExtendIntake extendIntake(Intake intakeSubsystem, double r) { return new ExtendIntake(intakeSubsystem, r); }

    // PERMANENTLY bind the drivebase subsystem to some double suppliers, usually joystick axis.
    public static SetDriveVector setDriveVector(Drivebase drivebaseSubsystem, DoubleSupplier x, DoubleSupplier y) {
        return new SetDriveVector(drivebaseSubsystem, x, y);
    }

    // PERMANENTLY bind the intake subsystem's gimble yaw rotation to some double suppliers
    public static RotateIntakeClaw rotateIntakeClaw(Intake intakeSubsystem, DoubleSupplier right, DoubleSupplier left) {
        return new RotateIntakeClaw(intakeSubsystem, right, left);
    }
}
