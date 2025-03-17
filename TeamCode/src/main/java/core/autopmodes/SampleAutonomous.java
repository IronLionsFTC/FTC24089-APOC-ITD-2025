package core.autopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.commands.CMD;
import core.math.Vector;
import core.subsystems.Intake;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous ( name = "SAMPLE AUTONOMOUS" )
public class SampleAutonomous extends CommandOpMode {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Follower follower;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry);

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);

        Constants.setConstants(FConstants.class, LConstants.class);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(Vector.cartesian(0, 0).pose(0));

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.followPath(this.follower, core.paths.SampleAutonomous.dumpPreload(), true).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(500),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem)
                )
        );
    }
}
