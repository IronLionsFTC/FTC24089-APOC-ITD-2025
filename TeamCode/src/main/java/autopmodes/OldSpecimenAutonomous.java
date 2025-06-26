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
import core.hardware.IndicatorLight;
import core.math.Vector;
import core.subsystems.Intake;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous ( name = "OLD SPECIMEN AUTONOMOUS" )
public class OldSpecimenAutonomous extends CommandOpMode {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Follower follower;
    private Limelight limelight;
    private Limelight.SampleState buffer;

    @Override
    public void initialize() {
        IndicatorLight light = new IndicatorLight(hardwareMap, "light");
        light.setColour(0.28);

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry, this.intakeSubsystem::forceDown);

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
                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.firstDump()).setSpeed(0.8).alongWith(
                                CMD.raiseSlidesForSpecimen(outtakeSubsystem)
                        ),
                        CMD.waitForProgress(follower, 0.95).andThen(CMD.clipSpecimen(outtakeSubsystem, 0.3)),
                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.firstSpike()).setSpeed(0.8).alongWith(
                                CMD.sleep(600).andThen(
                                        CMD.extendIntake(intakeSubsystem, 0.7, 697)
                                )
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.firstHp(), true).setSpeed(1),
                        CMD.releaseSample(intakeSubsystem),
                        CMD.sleep(200),

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.secondSpike(), true).setSpeed(0.8).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.7, 697)
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.secondHp()),
                        CMD.releaseSample(intakeSubsystem),
                        CMD.sleep(200),

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.thirdSpike(), true).setSpeed(1).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.7, 680)
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.thirdHp()),
                        CMD.releaseSample(intakeSubsystem),
                        CMD.sleep(200),

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.startCycling(), true).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.5, 300)
                        ),
                        CMD.sleep(500),
                        CMD.extendIntake(intakeSubsystem, 0.5, 690),
                        CMD.waitAndGrabSample(intakeSubsystem),

                        // ------------

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.goDumpA(), true).setSpeed(0.8).alongWith(
                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.raiseSlidesForSpecimen(outtakeSubsystem).andThen(
                                                CMD.waitForProgress(follower, 0.95).andThen(CMD.clipSpecimen(outtakeSubsystem, 0.3))
                                        )
                                )
                        ),

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.returnA(), true).setSpeed(1).alongWith(
                                CMD.sleep(800).andThen(CMD.extendIntake(intakeSubsystem, 0.5, 630))
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),

                        // ------------

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.goDumpB(), true).setSpeed(0.8).alongWith(
                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.raiseSlidesForSpecimen(outtakeSubsystem).andThen(
                                                CMD.waitForProgress(follower, 0.95).andThen(CMD.clipSpecimen(outtakeSubsystem, 0.3))
                                        )
                                )
                        ),

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.returnB(), true).setSpeed(1).alongWith(
                                CMD.sleep(800).andThen(CMD.extendIntake(intakeSubsystem, 0.5, 620))
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),

                        // ------------

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.goDumpC(), true).setSpeed(0.8).alongWith(
                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.raiseSlidesForSpecimen(outtakeSubsystem).andThen(
                                                CMD.waitForProgress(follower, 0.95).andThen(CMD.clipSpecimen(outtakeSubsystem, 0.3))
                                        )
                                )
                        ),

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.returnC(), true).setSpeed(1).alongWith(
                                CMD.sleep(800).andThen(CMD.extendIntake(intakeSubsystem, 0.5, 610))
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),

                        // ------------

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.goDumpD(), true).setSpeed(0.8).alongWith(
                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.raiseSlidesForSpecimen(outtakeSubsystem).andThen(
                                                CMD.waitForProgress(follower, 0.95).andThen(CMD.clipSpecimen(outtakeSubsystem, 0.3))
                                        )
                                )
                        ),

                        // ------------

                        CMD.followPath(follower, core.paths.OldSpecimenAutonomous.park()).setSpeed(1)
                )
        );

    }
}
