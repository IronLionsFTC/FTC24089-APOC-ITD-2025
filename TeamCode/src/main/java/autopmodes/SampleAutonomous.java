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

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.commands.CMD;
import core.computerVision.Limelight;
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
        this.buffer = new Limelight.SampleState();

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.followPath(this.follower, core.paths.SampleAutonomous.dumpPreload(), true).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(400),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.firstPreplaced(), true)
                                .alongWith(CMD.extendIntake(intakeSubsystem)
                        ),
                        CMD.sleep(400),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.firstDump(), true).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(400),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(CMD.extendIntake(intakeSubsystem)).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomous.secondPreplaced(), true)
                        ),
                        CMD.sleep(400),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.secondDump(), true).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(400),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.thirdGrab()).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.3, 0)
                        ),
                        CMD.sleep(400),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.thirdDump()).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(400),
                        CMD.slamDunkSample(outtakeSubsystem),





                        CMD.followPath(follower, core.paths.SampleAutonomous.goToSub()).setSpeed(1).alongWith(
                                CMD.sleep(1500).andThen(CMD.extendIntake(intakeSubsystem))
                        ),

                        CMD.searchForever(follower).raceWith(
                                CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false)
                        ),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.sleep(400),
                        CMD.grabSample(intakeSubsystem),
                        CMD.sleep(200),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                            CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ).alongWith(
                            CMD.sleep(300).andThen(CMD.followPath(follower, core.paths.SampleAutonomous.goToBasket()).setSpeed(1))
                        ),
                        CMD.sleep(400),
                        CMD.slamDunkSample(outtakeSubsystem),






                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomous.goToSub()).setSpeed(1).alongWith(
                                        CMD.sleep(1600).andThen(CMD.extendIntake(intakeSubsystem))
                                )
                        ),
                        CMD.searchForever(follower).raceWith(
                                CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false)
                        ),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.sleep(400),
                        CMD.grabSample(intakeSubsystem),
                        CMD.sleep(200),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ).alongWith(
                                CMD.sleep(300).andThen(CMD.followPath(follower, core.paths.SampleAutonomous.goToBasket()).setSpeed(1))
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem),







                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomous.goToSub()).setSpeed(1).alongWith(
                                        CMD.sleep(1600).andThen(CMD.extendIntake(intakeSubsystem))
                                )
                        ),
                        CMD.searchForever(follower).raceWith(
                                CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false)
                        ),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.sleep(400),
                        CMD.grabSample(intakeSubsystem),
                        CMD.sleep(200),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ).alongWith(
                                CMD.sleep(300).andThen(CMD.followPath(follower, core.paths.SampleAutonomous.goToBasket()).setSpeed(1))
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem),




                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomous.goToSub()).setSpeed(1).alongWith(
                                        CMD.sleep(1600).andThen(CMD.extendIntake(intakeSubsystem))
                                )
                        ),
                        CMD.searchForever(follower).raceWith(
                                CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false)
                        ),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.sleep(400),
                        CMD.grabSample(intakeSubsystem),
                        CMD.sleep(200),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ).alongWith(
                                CMD.sleep(300).andThen(CMD.followPath(follower, core.paths.SampleAutonomous.goToBasket()).setSpeed(1))
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem),



                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomous.goToSub()).setSpeed(1).alongWith(
                                        CMD.sleep(1600).andThen(CMD.extendIntake(intakeSubsystem))
                                )
                        ),
                        CMD.searchForever(follower).raceWith(
                                CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false)
                        ),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.sleep(400),
                        CMD.grabSample(intakeSubsystem),
                        CMD.sleep(200),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ).alongWith(
                                CMD.sleep(300).andThen(CMD.followPath(follower, core.paths.SampleAutonomous.goToBasket()).setSpeed(1))
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem)
                )
        );
    }
}
