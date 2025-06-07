package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import core.commands.CMD;
import core.computerVision.Limelight;
import core.math.Vector;
import core.subsystems.Intake;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Stationary CV test")
public class StationaryCVTest extends CommandOpMode {
    private Follower follower;
    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Limelight limelight;
    private Limelight.SampleState buffer;

    @Override
    public void initialize() {

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry);

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(Vector.cartesian(0, 0).pose(0));

        this.limelight = new Limelight(hardwareMap, Limelight.Targets.YellowOnly);
        this.buffer = new Limelight.SampleState();

        GamepadEx gamepad = new GamepadEx(gamepad1);
        GamepadButton button = gamepad.getGamepadButton(GamepadKeys.Button.X);

        schedule(
                new RunCommand(follower::update),
                new RunCommand(telemetry::update),
                new SequentialCommandGroup (
                        CMD.sleepUntil(this::opModeIsActive),

                        CMD.sleep(1000),

                        CMD.raiseLimelight(limelight),

                        CMD.sleep(1000),

                        CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false),
                        CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer, telemetry).alongWith(
                                CMD.alignClaw(intakeSubsystem, buffer)
                        ),

                        CMD.sleep(500),
                        CMD.grabSample(intakeSubsystem),
                        CMD.sleep(100),
                        CMD.grabSampleAbortIfEmpty(
                                intakeSubsystem,
                                outtakeSubsystem,
                                limelight,
                                buffer,
                                telemetry,
                                follower
                        ),

                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),

                        CMD.resetCV(buffer),

                        CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false),
                        CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer, telemetry).alongWith(
                                CMD.alignClaw(intakeSubsystem, buffer)
                        ),

                        CMD.sleep(500),
                        CMD.grabSample(intakeSubsystem),
                        CMD.sleep(100),
                        CMD.grabSampleAbortIfEmpty(
                                intakeSubsystem,
                                outtakeSubsystem,
                                limelight,
                                buffer,
                                telemetry,
                                follower
                        ),

                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),

                        CMD.moveAbsolute(follower, Vector.cartesian(0, 0), true)
                )
        );
    }
}
