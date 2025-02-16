package core.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import core.subsystems.Drivebase;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class CMD {

    // Extensions of builtin commands
    public static Command sleep(long ms) { return new WaitCommand(ms); }
    public static Command sleepUntil(BooleanSupplier condition) { return new WaitUntilCommand(condition); }

    // Instant Commands
    public static InstantCommand rotateCCW(Drivebase drivebaseSubsystem) {
        return new InstantCommand(drivebaseSubsystem::rotate45DegreesCCW);
    }
    public static InstantCommand rotateCW(Drivebase drivebaseSubsystem) {
        return new InstantCommand(drivebaseSubsystem::rotate45DegreesCW);
    }

    // Extends the intake, automatically folding down the claw and rotating to 0 or x degrees.
    public static ExtendIntake extendIntake(Intake intakeSubsystem) { return new ExtendIntake(intakeSubsystem, (double)0); }
    public static ExtendIntake extendIntake(Intake intakeSubsystem, double r) { return new ExtendIntake(intakeSubsystem, r); }

    // Cycle intake state machine
    public static InstantCommand teleopIntakeCycle(Intake intakeSubsystem) {
        return new InstantCommand(intakeSubsystem::nextState);
    }
    // Cycle outtake state machine
    public static InstantCommand teleopOuttakeCycle(Outtake outtakeSubsystem) {
        return new InstantCommand(outtakeSubsystem::nextState);
    }

    // PERMANENTLY perform automatic transfer, targetted for teleop but could be used in auto
    public static Command teleopAutomaticTransfer(Intake intakeSubsystem, Outtake outtakeSubsystem) { return new TeleopAutomaticTransfer(intakeSubsystem, outtakeSubsystem); }

    // Override control (Y button) does different things to different subsystems depending on the state.
    // General overview:
    //
    // Cancels grab (drops sample)

    // PERMANENTLY bind the drivebase subsystem to some double suppliers, usually joystick axis.
    public static SetDriveVector setDriveVector(Drivebase drivebaseSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        return new SetDriveVector(drivebaseSubsystem, x, y, r);
    }

    // PERMANENTLY bind the intake subsystem's gimble yaw rotation to some double suppliers
    public static RotateIntakeClaw rotateIntakeClaw(Intake intakeSubsystem, DoubleSupplier right, DoubleSupplier left) {
        return new RotateIntakeClaw(intakeSubsystem, right, left);
    }
}
