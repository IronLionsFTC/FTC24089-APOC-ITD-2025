package teleopmodes;

// Package imports from FTC libaries

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.BooleanSupplier;

import core.commands.CMD;
import core.commands.IronLionsInterrupt;
import core.commands.ToggleExtension;
import core.computerVision.Limelight;
import core.controls.Controls.Buttons;
import core.hardware.IndicatorLight;
import core.math.Vector;
import core.subsystems.Drivebase;
import core.subsystems.Intake;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "- RED Teleop", group = "Competition")
public class RedTeleop extends CommandOpMode {

    // Create subsystems
    private Drivebase drivebaseSubsystem;
    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Buttons buttons;
    private Follower follower;

    // CV handler and buffer for results
    private Limelight limelight;
    private Limelight.SampleState sampleState;

    private IndicatorLight light;

    @Override
    public void initialize() {

        // Load pedro tune
        Constants.setConstants(FConstants.class, LConstants.class);

        // initialize follower
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(Vector.cartesian(0, 0).pose(0));
        this.light = new IndicatorLight(hardwareMap, "light");

        // Intialize the rest of subsystems
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.buttons = new Buttons(gamepad1, gamepad2);
        BooleanSupplier zeroSlides = this.buttons.zeroSlides::get;
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry, this.light, zeroSlides);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry, this.intakeSubsystem::forceDown, zeroSlides);
        this.drivebaseSubsystem = new Drivebase(hardwareMap, this.telemetry, this.intakeSubsystem::isSlidesExtended);

        this.limelight = new Limelight(hardwareMap, Limelight.Targets.RedAndYellow);
        this.limelight.raise();
        this.sampleState = new Limelight.SampleState();
        this.follower.setMaxPower(0.6);
        this.intakeSubsystem.setExtension(450);

        // IMPORTANT - Register SUBSYSTEMS that implement periodic
        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);

        // Link buttons to commands
        buttons.intakeCycle.whenPressed(CMD.teleopIntakeCycle(intakeSubsystem));
        buttons.outtakeCycle.whenPressed(CMD.teleopOuttakeCycle(outtakeSubsystem));
        buttons.rotateRight.whenPressed(CMD.rotateCW(drivebaseSubsystem));
        buttons.rotateLeft.whenPressed(CMD.rotateCCW(drivebaseSubsystem));
        buttons.override.whenPressed(CMD.teleopOverride(intakeSubsystem, outtakeSubsystem));
        buttons.toggleExtension.whenPressed(
                new ToggleExtension(this.intakeSubsystem)
        );

        // Spawn a command sequence to rotate the claw to align with a sample
        buttons.useCV.whenPressed(
                CMD.resetCV(sampleState).andThen(
                        new InstantCommand(limelight::disable)
                ).andThen(
                        new InstantCommand(limelight::enable)
                ).andThen(
                new IronLionsInterrupt(
                        CMD.disableDrivebase(drivebaseSubsystem).andThen(
                            CMD.scanForSample(limelight, sampleState, telemetry, follower, intakeSubsystem, false).andThen(
                                    (CMD.driveToSampleUseSlides(follower, intakeSubsystem, sampleState, telemetry).alongWith(
                                            CMD.alignClaw(intakeSubsystem, sampleState)
                                    )).andThen(
                                            CMD.resetCV(sampleState)
                                    ).andThen(
                                            CMD.shortWaitAndGrabSample(intakeSubsystem)
                                    )
                            )
                    ),
                this.buttons::interruptCV).andThen(
                        CMD.resetCV(sampleState)
                ).andThen(
                        new InstantCommand(limelight::disable)
                ).andThen(
                        CMD.enableDrivebase(drivebaseSubsystem)
                )
        ));

        // Emergency retract the intake
        buttons.emergencyIntakeRetract.whenPressed(
                CMD.retractIntake(intakeSubsystem)
        );

        // Schedule the command based opmode
        schedule(
                CMD.sleepUntil(this::opModeIsActive),
                new RunCommand(telemetry::update),
                new ParallelCommandGroup(
                        CMD.updateFollowerWhenDrivebaseDisabled(drivebaseSubsystem, follower, this.buttons::interruptCV),

                        // Switch between follower and manual control (follower only used to hold points against collision)
                        // This means if we get hit when stationary it will correct back when enabled
                        // could be used to do more CV things in future
                        CMD.constantlyUpdateFollower(follower, drivebaseSubsystem),

                        // Constantly grab the joystick position, scale it like so:
                        /*
                                            *
                         S                  *
                         p                 *
                         e                *
                         e             **
                         d          ***
                             *****
                             Joystick Position
                         */

                        // This makes it easier to do small movement with little joystick motion, but full speed still is full speed (1^2 = 1, 0.5^2 = 0.25)
                        CMD.setDriveVector(drivebaseSubsystem, intakeSubsystem, buttons.driveX, buttons.driveY, buttons.yaw),

                        // Rotate the claw when in certain states. This command is a wrapper for a bunch of smaller commands
                        CMD.rotateIntakeClaw(intakeSubsystem, buttons.rotateClawRight, buttons.rotateClawLeft),

                        // Optionally scheduled command that uses proximity sensors and encoders to confidently attempt transfer automatically
                        // If this is not scheduled driver is in control. During comp, if something fails and we can't fix it in time, this could
                        // be removed so that driver can control it. (IE if outtake sensor breaks or cable breaks, or physical alignment is changed)
                        CMD.teleopAutomaticTransfer(intakeSubsystem, outtakeSubsystem)

                        // Automatically open the claw when it grabs and nothing is there.
                        // CMD.autoRejectionRunCommand(intakeSubsystem, telemetry)
                )
        );
    }
}
