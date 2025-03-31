package autopmodes;

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
import core.computerVision.Limelight;
import core.math.Vector;
import core.subsystems.Intake;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous ( name = "SAMPLE AUTONOMOUS V2" )
public class SampleAutonomousV2 extends CommandOpMode {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Follower follower;
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
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(Vector.cartesian(0, 0).pose(0));

        this.limelight = new Limelight(hardwareMap, Limelight.Targets.YellowOnly);
        this.buffer = new Limelight.SampleState(0, Vector.cartesian(0, 0));

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.firstDumpAndPickup())
                        ),

                        CMD.sleep(100),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.sleep(100),

                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.5, 0.04).andThen(CMD.grabSample(intakeSubsystem))
                        ),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.secondDumpAndPickup())
                        ),

                        CMD.sleep(100),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.sleep(100),

                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.5, 0.07).andThen(CMD.grabSample(intakeSubsystem))
                        ),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem),

                        CMD.sleep(100),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.sleep(100),

                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.thirdDumpAndPickup())
                        ).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.7, 0.0)
                        ),

                        CMD.sleep(300),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).alongWith(
                        )
                )
        );
    }
}
