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
import core.parameters.PositionalBounds;
import core.paths.SampleAutonomousV5;
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
    public static ExtendIntake extendIntake(Intake intakeSubsystem) { return new ExtendIntake(intakeSubsystem, 0.5, PositionalBounds.SlidePositions.IntakePositions.extended); }
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
        return new InstantCommand(sampleState::reset);
    }
    public static InstantCommand disableDrivebase(Drivebase drivebaseSubsystem) {
        return new InstantCommand(drivebaseSubsystem::disable);
    }

    public static InstantCommand enableDrivebase(Drivebase drivebaseSubsystem) {
        return new InstantCommand(drivebaseSubsystem::enable);
    }

    public static ConstantlyUpdateFollower constantlyUpdateFollower(Follower follower, Drivebase drivebaseSubsystem) {
        return new ConstantlyUpdateFollower(follower, drivebaseSubsystem);
    }

    public static Command goToSubForCycles(Follower follower, Limelight.SampleState buffer, Limelight.SampleState cached) {
        return new SequentialCommandGroup(
                CMD.basketToSubCached(follower, cached)
        );
    }

    public static Command subCycle(
            Follower follower,
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Limelight.SampleState cached,
            Telemetry telemetry,
            IndicatorLight light
    ) {
        return new SequentialCommandGroup(
                CMD.basketToSubCached(follower, cached),
                CMD.grabSampleForSubCycles(
                        follower,
                        intakeSubsystem,
                        outtakeSubsystem,
                        limelight,
                        buffer,
                        cached,
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
            Limelight.SampleState cached,
            Telemetry telemetry,
            IndicatorLight light
    ) {
        return new SequentialCommandGroup(
                CMD.subToCvCached(follower, cached),
                CMD.sleep(200).alongWith(CMD.resetCV(cached)).alongWith(CMD.resetCV(buffer)),
                CMD.scanForTwoSamples(limelight, telemetry, follower, buffer, cached, intakeSubsystem),
                CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer, telemetry).alongWith(
                        CMD.alignClaw(intakeSubsystem, buffer)
                ),
                CMD.shortWaitAndGrabSample(intakeSubsystem),
                CMD.sleep(200),
                CMD.grabSampleAbortIfEmpty(intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, follower),
                CMD.goToBasketForSubCycles(follower, intakeSubsystem, outtakeSubsystem)
        );
    }

    public static Command goToBasketForSubCycles(Follower follower, Intake intakeSubsystem, Outtake outtakeSubsystem) {
        return new SequentialCommandGroup(
                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                CMD.waitForProgress(follower, 0.97).andThen(
                                        CMD.slamDunkSample(outtakeSubsystem)
                                )
                        )
                ).alongWith(
                        //CMD.followPath(follower, core.paths.SampleAutonomousV5.subToBasket()).setSpeed(1)
                        CMD.followPath(follower, core.paths.SampleAutonomousV5.testCV(follower)).setSpeed(1)
                )
        );
    }

    public static InstantCommand light(IndicatorLight light, double colour) {
        return new InstantCommand(()->light.setColour(colour));
    }

    public static DriveToSampleUseSlides driveToSampleUseSlides(Follower follower, Intake intakeSubsystem, Limelight.SampleState buffer, Telemetry telemetry) {
        return new DriveToSampleUseSlides(follower, intakeSubsystem, buffer, telemetry);
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

    public static RaiseLimelight raiseLimelight(Limelight limelight) {
        return new RaiseLimelight(limelight);
    }

    public static HideLimelight hideLimelight(Limelight limelight) {
        return new HideLimelight(limelight);
    }

    public static RetractIntake retractIntake(Intake intakeSubsystem) {
        return new RetractIntake(intakeSubsystem);
    }

    public static GrabSampleAbortIfEmpty grabSampleAbortIfEmpty(
        Intake intakeSubsystem,
        Outtake outtakeSubsystem,
        Limelight limelight,
        Limelight.SampleState buffer,
        Telemetry telemetry,
        Follower follower
    ) {
        return new GrabSampleAbortIfEmpty(
                intakeSubsystem,
                outtakeSubsystem,
                limelight,
                buffer,
                telemetry,
                follower
        );
    }

    public static AutoRejectionRunCommand autoRejectionRunCommand(Intake intakeSubsystem, Telemetry telemetry) {
        return new AutoRejectionRunCommand(intakeSubsystem, telemetry);
    }

    public static ExtendSlidesForSample extendSlidesForSample(Intake intakeSubsystem, Limelight.SampleState buffer) {
        return new ExtendSlidesForSample(intakeSubsystem, buffer);
    }

    public static WaitAndGrabSample waitAndGrabSample(Intake intakeSubsystem) {
        return new WaitAndGrabSample(intakeSubsystem);
    }

    public static RetryAndRepeat retryAndRepeat(
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Telemetry telemetry,
            Follower follower
    ) {
        return new RetryAndRepeat(
                intakeSubsystem,
                outtakeSubsystem,
                follower,
                telemetry,
                limelight,
                buffer
        );
    }

    public static Command waitForStartWithPreloadWarning(IndicatorLight light, Intake intakeSubsystem, BooleanSupplier condition) {
        return new PreloadWarning(intakeSubsystem, light, condition);
    }

    public static ScanForTwoSamples scanForTwoSamples(Limelight limelight, Telemetry telemetry, Follower follower, Limelight.SampleState buffer, Limelight.SampleState cache, Intake intakeSubsystem) {
        return new ScanForTwoSamples(limelight, buffer, cache, telemetry, follower, intakeSubsystem, false);
    }

    public static UpdateFollowerWhenDrivebaseDisabled updateFollowerWhenDrivebaseDisabled(Drivebase drivebase, Follower follower, BooleanSupplier joystick) {
        return new UpdateFollowerWhenDrivebaseDisabled(drivebase, follower, joystick);
    }

    public static IronLionsInterrupt ironLionsInterrupt(Command command, BooleanSupplier condition) {
        return new IronLionsInterrupt(command, condition);
    }

    public static ShortWaitAndGrabSample shortWaitAndGrabSample(Intake intakeSubsystem) {
        return new ShortWaitAndGrabSample(intakeSubsystem);
    }

    public static BasketToSubCached basketToSubCached(Follower follower, Limelight.SampleState cached) {
        return new BasketToSubCached(follower, cached);
    }

    public static SubToCvCached subToCvCached(Follower follower, Limelight.SampleState cached) {
        return new SubToCvCached(follower, cached);
    }

    public static RetractIntakeSlightly retractIntakeSlightly(Intake intakeSubsystem) {
        return new RetractIntakeSlightly(intakeSubsystem);
    }

    public static JerkAndStartRepeating jerkAndStartRepeating(
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Telemetry telemetry,
            Follower follower
    ) {
        return new JerkAndStartRepeating(
                intakeSubsystem,
                outtakeSubsystem,
                limelight,
                buffer,
                telemetry,
                follower
        );
    }

    public static Command waitForProgress(Follower follower, double progress) {
        return new WaitUntilCommand(() -> follower.getCurrentTValue() >= progress);
    }

    public static Command sampleAuto(
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Follower follower,
            IndicatorLight light,
            BooleanSupplier opModeIsActive,
            Limelight limelight,
            Limelight.SampleState buffer,
            Limelight.SampleState cache,
            Telemetry telemetry
    ) {
        return new SequentialCommandGroup(
                CMD.waitForStartWithPreloadWarning(light, intakeSubsystem, opModeIsActive),

                /*
                CMD.followPath(follower, core.paths.SampleAutonomousV5.firstDumpAndPickup()).setSpeed(0.7).alongWith(
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                CMD.waitForProgress(follower, 0.72).andThen(CMD.slamDunkSample(outtakeSubsystem))
                        )
                ).alongWith(
                        CMD.sleep(800).andThen(CMD.extendIntake(intakeSubsystem, 0.55, 700).andThen(
                                CMD.shortWaitAndGrabSample(intakeSubsystem).andThen(
                                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
                                )
                        ))
                ),
                */

                CMD.followPath(follower, core.paths.SampleAutonomousV5.testFirstDump()).setSpeed(1).alongWith(
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                CMD.waitForProgress(follower, 0.72).andThen(
                                        CMD.slamDunkSample(outtakeSubsystem)
                                )
                        )
                ), CMD.followPath(follower, core.paths.SampleAutonomousV5.testFirstPickup()).setSpeed(1).alongWith(
                        CMD.extendIntake(intakeSubsystem, 0.55, 780).andThen(
                                CMD.shortWaitAndGrabSample(intakeSubsystem).andThen(
                                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
                                )
                        )
                ),

                CMD.followPath(follower, core.paths.SampleAutonomousV5.secondDumpAndPickup()).alongWith(
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                CMD.sleep(300).andThen(CMD.slamDunkSample(outtakeSubsystem))
                        )
                ).alongWith(
                        CMD.sleep(200).andThen(CMD.extendIntake(intakeSubsystem, 0.5, 600).andThen(
                                CMD.shortWaitAndGrabSample(intakeSubsystem).andThen(
                                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
                                )
                        ))
                ),

                CMD.raiseSlidesForSampleDump(outtakeSubsystem).alongWith(
                        CMD.followPath(follower, core.paths.SampleAutonomousV5.secondDumpAndPickup())
                ),
                CMD.sleep(300),
                CMD.slamDunkSample(outtakeSubsystem),

                CMD.followPath(follower, core.paths.SampleAutonomousV5.thirdDumpAndPickup()).alongWith(
                        CMD.sleep(300).andThen(CMD.extendIntake(intakeSubsystem, 0.43, 670))
                ),
                CMD.sleep(300).andThen(CMD.grabSample(intakeSubsystem)),

                CMD.followPath(follower, core.paths.SampleAutonomousV5.lastDump()).alongWith(
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        )
                ),
                CMD.sleep(300),
                CMD.slamDunkSample(outtakeSubsystem),

                CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, cache, telemetry, light),
                CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, cache, telemetry, light),
                CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, cache, telemetry, light),
                CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, cache, telemetry, light)
        );
    }

    public static GoToSpecimenIntake goToSpecimenIntake(Outtake outtakeSubsystem) {
        return new GoToSpecimenIntake(outtakeSubsystem);
    }

    public static SpecimenIntakeBySensor specimenIntakeBySensor(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        return new SpecimenIntakeBySensor(intakeSubsystem, outtakeSubsystem);
    }

    public static RetractIntakeAndTransferHalf retractIntakeAndTransferHalf(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        return new RetractIntakeAndTransferHalf(intakeSubsystem, outtakeSubsystem);
    }

    public static InstantCommand intakeFull(Intake intakeSubsystem) {
        return new InstantCommand(intakeSubsystem::full);
    }
}