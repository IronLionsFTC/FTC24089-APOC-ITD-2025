package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
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

        Limelight limelight = new Limelight(hardwareMap);
        Limelight.SampleState buffer = new Limelight.SampleState(0, Vector.cartesian(0, 0));

        schedule(
                new RunCommand(follower::update),
                new ParallelCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.extendIntake(intakeSubsystem),
                        CMD.scanForSample(follower, limelight, buffer),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.sleep(500)
                )
        );
    }
}
