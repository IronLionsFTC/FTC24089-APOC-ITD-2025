package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import core.commands.CMD;
import core.math.Vector;
import core.paths.SampleAutonomousV2;
import core.subsystems.Intake;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "AutonomousPickupTest")
public class AutonomousPickupTest extends CommandOpMode {
    private Follower follower;
    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry, this.intakeSubsystem::forceDown);

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(Vector.cartesian(0, 0).pose(0));

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.followPath(follower, SampleAutonomousV2.firstDumpAndPickup()).alongWith(
                                CMD.extendIntake(intakeSubsystem)
                        ),
                        CMD.sleep(5000),
                        CMD.followPath(follower, SampleAutonomousV2.secondDumpAndPickup()),
                        CMD.sleep(5000),
                        CMD.followPath(follower, SampleAutonomousV2.thirdDumpAndPickup())
                )
        );
    }
}
