package core.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import core.computerVision.Limelight;
import core.hardware.IndicatorLight;
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
    public static ExtendIntake extendIntake(Intake intakeSubsystem) { return new ExtendIntake(intakeSubsystem, 0.5, 0.0); }
    public static ExtendIntake extendIntake(Intake intakeSubsystem, double r, double e) { return new ExtendIntake(intakeSubsystem, r, e); }

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

    // Specimen Cycle
    public static RaiseSlidesForSpecimen raiseSlidesForSpecimen(Outtake outtakeSubsystem) { return new RaiseSlidesForSpecimen(outtakeSubsystem); }
    public static ClipSpecimen clipSpecimen(Outtake outtakeSubsystem, double time) { return new ClipSpecimen(outtakeSubsystem, time); }

    // PERMANENTLY perform automatic transfer, targetted for teleop but could be used in auto
    public static Command teleopAutomaticTransfer(Intake intakeSubsystem, Outtake outtakeSubsystem) { return new TeleopAutomaticTransfer(intakeSubsystem, outtakeSubsystem); }

    public static Command teleopOverride(Intake intakeSubsystem, Outtake outtakeSubsystem) { return new TeleopOverride(intakeSubsystem, outtakeSubsystem); }

    // Override control (Y button) does different things to different subsystems depending on the state.
    // General overview:
    //
    // Cancels grab (drops sample)

    // PERMANENTLY bind the drivebase subsystem to some double suppliers, usually joystick axis.
    public static SetDriveVector setDriveVector(Drivebase drivebaseSubsystem, Intake intakeSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        return new SetDriveVector(drivebaseSubsystem, intakeSubsystem, x, y, r);
    }

    // PERMANENTLY bind the intake subsystem's gimble yaw rotation to some double suppliers
    public static RotateIntakeClaw rotateIntakeClaw(Intake intakeSubsystem, DoubleSupplier right, DoubleSupplier left) {
        return new RotateIntakeClaw(intakeSubsystem, right, left);
    }

    // ---------------- PEDRO COMMANDS --------------------------------------
    public static MoveRelative moveRelative(Follower follower, Vector position, boolean holdEnd) { return new MoveRelative(follower, position, holdEnd); }
    public static MoveAbsolute moveAbsolute(Follower follower, Vector position, boolean holdEnd) { return new MoveAbsolute(follower, position, holdEnd); }

    public static FollowPath followPath(Follower follower, PathChain path) { return new FollowPath(follower, path); }
    public static FollowPath followPath(Follower follower, PathChain path, boolean holdEnd) { return new FollowPath(follower, path, holdEnd); }

    public static DriveToSample driveToSample(Follower follower, Limelight.SampleState buffer) { return new DriveToSample(follower, buffer); }

    // --------------- COMPUTER VISION ----------------------------------
    public static ScanForSample scanForSample(Limelight limelight, Limelight.SampleState buffer, Telemetry telemetry, Follower follower, Intake intakeSubsystem, boolean isSub) {
        return new ScanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, isSub);
    }
    public static AlignClaw alignClaw(Intake intakeSubsystem, Limelight.SampleState buffer) { return new AlignClaw(intakeSubsystem, buffer); }
    public static SearchForever searchForever(Follower follower) { return new SearchForever(follower); }
    public static InstantCommand resetCV(Limelight.SampleState sampleState) {
        return new InstantCommand(() -> {
            sampleState.angle = 0;
            sampleState.center = Vector.cartesian(0, 0);
            sampleState.intakeTilt = 0;
            sampleState.slidePosition = 0;
            sampleState.robotRotation = 0;
            sampleState.robotPosition = Vector.cartesian(0, 0);
        });
    }
    public static InstantCommand disableDrivebase(Drivebase drivebaseSubsystem) {
        return new InstantCommand(() -> {
            drivebaseSubsystem.disable();
        });
    }

    public static InstantCommand enableDrivebase(Drivebase drivebaseSubsystem) {
        return new InstantCommand(() -> {
            drivebaseSubsystem.enable();
        });
    }

    public static ConstantlyUpdateFollower constantlyUpdateFollower(Follower follower, Drivebase drivebaseSubsystem) {
        return new ConstantlyUpdateFollower(follower, drivebaseSubsystem);
    }

    public static Command goToSubForCycles(Follower follower, Limelight.SampleState buffer) {
        return new SequentialCommandGroup(
                CMD.resetCV(buffer),
                CMD.followPath(follower, core.paths.SampleAutonomousV2.basketToSub()).setSpeed(1)
        );
    }

    public static Command subCycle(
            Follower follower,
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Telemetry telemetry,
            IndicatorLight light
    ) {
        return new SequentialCommandGroup(
                CMD.goToSubForCycles(follower, buffer),
                CMD.grabSampleForSubCycles(
                        follower,
                        intakeSubsystem,
                        outtakeSubsystem,
                        limelight,
                        buffer,
                        telemetry,
                        light
                )
        );
    }

    public static Command grabSampleForSubCycles(
            Follower follower,
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Telemetry telemetry,
            IndicatorLight light
    ) {
        return new SequentialCommandGroup(
                CMD.followPath(follower, core.paths.SampleAutonomousV2.subToCV()).setSpeed(1).alongWith(
                        CMD.sleep(500).andThen(CMD.extendIntake(intakeSubsystem, 0.5, 0.4))
                ),
                CMD.setTilt(intakeSubsystem, 0.1),
                CMD.sleep(500),
                CMD.light(light, 0.28),
                CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false).tilt(0.1),
                CMD.light(light, 0.5),
                CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer),
                CMD.light(light, 0.611),
                CMD.alignClaw(intakeSubsystem, buffer),
                CMD.light(light, 0.333),
                CMD.sleep(600),
                CMD.grabSample(intakeSubsystem),
                CMD.light(light, 0.388),
                new RecursiveSubIntake(
                        follower,
                        intakeSubsystem,
                        outtakeSubsystem,
                        limelight,
                        buffer,
                        telemetry,
                        light
                )
        );
    }

    public static Command goToBasketForSubCycles(Follower follower, Intake intakeSubsystem, Outtake outtakeSubsystem) {
        return new SequentialCommandGroup(
                CMD.sleep(500).andThen(CMD.followPath(follower, core.paths.SampleAutonomousV2.subToBasket())
                                .setSpeed(1)).alongWith(
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.sleep(500).andThen(CMD.raiseSlidesForSampleDump(outtakeSubsystem))
                        )
                ),
                CMD.sleep(200),
                CMD.slamDunkSample(outtakeSubsystem),
                CMD.sleep(200)
        );
    }

    public static InstantCommand light(IndicatorLight light, double colour) {
        return new InstantCommand(()->light.setColour(colour));
    }

    public static DriveToSampleUseSlides driveToSampleUseSlides(Follower follower, Intake intakeSubsystem, Limelight.SampleState buffer) {
        return new DriveToSampleUseSlides(follower, intakeSubsystem, buffer);
    }

    public static InstantCommand setTilt(Intake intakeSubsystem, double tilt) {
        return new InstantCommand(()->intakeSubsystem.setTilt(tilt));
    }

    public static InstantCommand releaseSample(Intake intakeSubsystem) {
        return new InstantCommand(
                intakeSubsystem::cancelGrab
        );
    }

    public static InstantCommand setClawRotation(Intake intakeSubsystem, double angle) {
        return new InstantCommand(
                ()->intakeSubsystem.setIntakeClawRotation(angle)
        );
    }
}