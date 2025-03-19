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

        this.limelight = new Limelight(hardwareMap);
        this.buffer = new Limelight.SampleState(0, Vector.cartesian(0, 0));

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.followPath(this.follower, core.paths.SampleAutonomous.dumpPreload(), true).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.firstPreplaced(), true)
                            .alongWith(CMD.extendIntake(intakeSubsystem)),
                        CMD.sleep(300),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomous.firstDump(), true)
                        ),
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem),
                        CMD.sleep(200),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(CMD.extendIntake(intakeSubsystem)).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomous.secondPreplaced(), true)
                        ),
                        CMD.sleep(300),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.secondDump(), true).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(200),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(CMD.followPath(follower, core.paths.SampleAutonomous.thirdGrab())),
                        CMD.extendIntake(intakeSubsystem, 0.3),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.thirdDump()).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(CMD.followPath(follower, core.paths.SampleAutonomous.goToSub())),

                        CMD.extendIntake(intakeSubsystem),
                        CMD.moveRelative(follower, Vector.cartesian(3, 10), true).raceWith(
                                CMD.scanForSample(follower, limelight, buffer, telemetry)
                        ),
                        CMD.sleep(300),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.sleep(300),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.goToBasket()).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomous.goToSub())
                        ),
                        CMD.extendIntake(intakeSubsystem),
                        CMD.moveRelative(follower, Vector.cartesian(3, 10), true).raceWith(
                                CMD.scanForSample(follower, limelight, buffer, telemetry)
                        ),
                        CMD.sleep(300),
                        CMD.driveToSample(follower, buffer),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.sleep(300),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SampleAutonomous.goToBasket()).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem)
                )
        );
    }
}
