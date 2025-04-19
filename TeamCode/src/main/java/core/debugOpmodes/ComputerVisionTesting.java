package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.commands.CMD;
import core.computerVision.Limelight;
import core.hardware.CachedServo;
import core.math.Vector;
import core.parameters.HardwareParameters;
import core.subsystems.Intake;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp ( name = "COMPUTER VISION POSITION TEST")
public class ComputerVisionTesting extends CommandOpMode {
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

        schedule(
                new RunCommand(follower::update),
                new RunCommand(telemetry::update),
                new SequentialCommandGroup (
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.moveAbsolute(follower, Vector.cartesian(0, 14), false),
                        CMD.extendIntake(intakeSubsystem),
                        CMD.searchForever(follower).raceWith(
                            CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false)
                        ),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.sleep(300),
                        CMD.grabSample(intakeSubsystem),
                        CMD.sleep(300),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.moveAbsolute(follower, Vector.cartesian(0, 0), true)
                )
        );
    }
}
