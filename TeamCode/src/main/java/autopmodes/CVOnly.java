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

@Autonomous ( name = "CV ONLY --------" )
public class CVOnly extends CommandOpMode {

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
        this.buffer = new Limelight.SampleState(0, Vector.cartesian(0, 0), Vector.cartesian(0, 0), 0);

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),

                        CMD.followPath(follower, core.paths.SampleAutonomousV2.firstDumpAndPickup()).setSpeed(1),

                        CMD.followPath(follower, core.paths.SampleAutonomousV2.basketToSub()).setSpeed(1),
                        CMD.followPath(follower, core.paths.SampleAutonomousV2.subToCV()).setSpeed(0.5).alongWith(
                                CMD.extendIntake(intakeSubsystem)
                        ),
                        CMD.searchForever(follower).raceWith(
                                CMD.scanForSample(limelight, buffer, telemetry, follower)
                        ),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.sleep(300),
                        CMD.grabSample(intakeSubsystem),
                        CMD.sleep(300),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomousV2.subToBasket()).setSpeed(1).alongWith(
                                CMD.sleep(1500).andThen(CMD.raiseSlidesForSampleDump(outtakeSubsystem))
                        ),
                        CMD.sleep(200),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.sleep(200)
                )
        );
    }
}
