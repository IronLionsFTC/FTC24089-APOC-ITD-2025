package core.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import core.math.Vector;
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

    // Assumes intake is extended and claw is down and ready to grab.
    public static GrabSample grabSample(Intake intakeSubsystem) { return new GrabSample(intakeSubsystem); }

    // Assumes the intake is currently holding a sample at full extension and intake is down.
    public static RetractIntakeAndTransfer retractIntakeAndTransfer(Intake intakeSubsystem, Outtake outtakeSubsystem) { return new RetractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem); }

    // Assumes that the intake is currently retracted and that the outtake is gripping the sample in transfer position.
    public static RaiseSlidesForSampleDump raiseSlidesForSampleDump(Outtake outtakeSubsystem) { return new RaiseSlidesForSampleDump(outtakeSubsystem); }

    // If the outtake is already up and ready to drop, then drop the sample.
    public static SlamDunkSample slamDunkSample(Outtake outtakeSubsystem) { return new SlamDunkSample(outtakeSubsystem); }

    // Cycle intake state machine
    public static InstantCommand teleopIntakeCycle(Intake intakeSubsystem) {
        return new InstantCommand(intakeSubsystem::nextState);
    }
    // Cycle outtake state machine
    public static InstantCommand teleopOuttakeCycle(Outtake outtakeSubsystem) {
        return new InstantCommand(outtakeSubsystem::nextState);
    }

    // Wait until outtake down for carefully timed actions
    public static WaitUntilOuttakeDown waitUntilOuttakeDown(Outtake outtakeSubsystem) {
        return new WaitUntilOuttakeDown(outtakeSubsystem);
    }

    // PERMANENTLY perform automatic transfer, targetted for teleop but could be used in auto
    public static Command teleopAutomaticTransfer(Intake intakeSubsystem, Outtake outtakeSubsystem) { return new TeleopAutomaticTransfer(intakeSubsystem, outtakeSubsystem); }

    public static Command teleopOverride(Intake intakeSubsystem, Outtake outtakeSubsystem) { return new TeleopOverride(intakeSubsystem, outtakeSubsystem); }

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

    // ---------------- PEDRO COMMANDS --------------------------------------
    public static MoveRelative moveRelative(Follower follower, Vector position, boolean holdEnd) { return new MoveRelative(follower, position, holdEnd); }
    public static MoveAbsolute moveAbsolute(Follower follower, Vector position, boolean holdEnd) { return new MoveAbsolute(follower, position, holdEnd); }
}
