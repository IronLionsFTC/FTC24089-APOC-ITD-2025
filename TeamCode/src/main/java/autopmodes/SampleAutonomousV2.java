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
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.firstDumpAndPickup()).setSpeed(1)
                        ),

                        CMD.sleep(100).andThen(
                            CMD.slamDunkSample(outtakeSubsystem).andThen(
                                CMD.sleep(300)
                            )
                        ).alongWith(
                            CMD.extendIntake(intakeSubsystem, 0.5, 0.1).andThen(CMD.grabSample(intakeSubsystem))
                        ),

                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.secondDumpAndPickup())
                        ),

                        CMD.sleep(300).andThen(
                                CMD.slamDunkSample(outtakeSubsystem).andThen(
                                        CMD.sleep(300)
                                )
                        ).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.5, 0.13).andThen(CMD.grabSample(intakeSubsystem))
                        ),

                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem),

                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.sleep(200),

                        CMD.followPath(follower, core.paths.SampleAutonomousV2.thirdDumpAndPickup()).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.3, 0.08)
                        ),

                        CMD.sleep(300),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),

                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.lastDump())
                        ),

                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.sleep(200),

                        CMD.followPath(follower, core.paths.SampleAutonomousV2.basketToSub()).setSpeed(1).alongWith( CMD.sleep(800).andThen(CMD.extendIntake(intakeSubsystem)) ),
                        CMD.followPath(follower, core.paths.SampleAutonomousV2.subToCV()).setSpeed(1),
                        CMD.searchForever(follower).raceWith(CMD.scanForSample(limelight, buffer, telemetry)),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(CMD.sleep(500).andThen(CMD.raiseSlidesForSampleDump(outtakeSubsystem))).alongWith(
                                CMD.sleep(500).andThen(CMD.followPath(follower, core.paths.SampleAutonomousV2.subToBasket()).setSpeed(1))
                        )
                )
        );
    }
}
