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

@Autonomous(name = "Stationary CV test, 2 samples")
public class StationaryTwoSampleCVTest extends CommandOpMode {
    private Follower follower;
    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Limelight limelight;

    private Limelight.SampleState buffer1;
    private Limelight.SampleState buffer2;

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
        this.buffer1 = new Limelight.SampleState();
        this.buffer2 = new Limelight.SampleState();

        GamepadEx gamepad = new GamepadEx(gamepad1);
        GamepadButton button = gamepad.getGamepadButton(GamepadKeys.Button.X);

        schedule(
                new RunCommand(follower::update),
                new RunCommand(telemetry::update),
                new SequentialCommandGroup (
                        CMD.sleepUntil(this::opModeIsActive),

                        new WaitUntilCommand(button::get),

                        CMD.extendIntake(intakeSubsystem, 0.5, 0.3),
                        CMD.setTilt(intakeSubsystem, 0.1),

                        new WaitUntilCommand(button::get),

                        CMD.scanForTwoSamples(limelight, buffer1, buffer2, telemetry, follower, intakeSubsystem, false).tilt(0.1),
                        CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer1),
                        CMD.alignClaw(intakeSubsystem, buffer1),
                        CMD.setTilt(intakeSubsystem, 0),
                        CMD.sleep(800),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),

                        CMD.extendIntake(intakeSubsystem, 0.5, 0.3),
                        CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer2),
                        CMD.alignClaw(intakeSubsystem, buffer2),
                        CMD.sleep(800),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.moveAbsolute(follower, Vector.cartesian(0, 0), true)
                )
        );
    }
}
