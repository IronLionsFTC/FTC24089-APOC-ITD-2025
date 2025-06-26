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

@Autonomous ( name = "-SPECIMEN AUTONOMOUS" )
public class SpecimenAutonomous extends CommandOpMode {

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
        this.limelight.raise();
        this.buffer = new Limelight.SampleState();

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.firstDump()).setSpeed(0.8).alongWith(
                                CMD.raiseSlidesForSpecimen(outtakeSubsystem)
                        ),
                        CMD.waitForProgress(follower, 0.95).andThen(CMD.clipSpecimen(outtakeSubsystem, 0.3)),

                        CMD.moveRelative(follower, Vector.cartesian(0, -5), true),
                        CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false),
                        CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer, telemetry),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransferHalf(intakeSubsystem, outtakeSubsystem).andThen(
                            CMD.goToSpecimenIntake(outtakeSubsystem)
                        ).alongWith(
                                CMD.followPath(follower, core.paths.SpecimenAutonomous.returnA())
                        ),
                        CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpA())
                        ),
                        CMD.clipSpecimen(outtakeSubsystem, 0.3),



                        CMD.moveRelative(follower, Vector.cartesian(0, -5), true),
                        CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false),
                        CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer, telemetry),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransferHalf(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.goToSpecimenIntake(outtakeSubsystem)
                        ).alongWith(
                                CMD.followPath(follower, core.paths.SpecimenAutonomous.returnA())
                        ),
                        CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpA())
                        ),
                        CMD.clipSpecimen(outtakeSubsystem, 0.3),



                        CMD.moveRelative(follower, Vector.cartesian(0, -5), true),
                        CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false),
                        CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer, telemetry),
                        CMD.alignClaw(intakeSubsystem, buffer),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransferHalf(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.goToSpecimenIntake(outtakeSubsystem)
                        ).alongWith(
                                CMD.followPath(follower, core.paths.SpecimenAutonomous.returnA())
                        ),
                        CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpA())
                        ),
                        CMD.clipSpecimen(outtakeSubsystem, 0.3),



                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnA()).alongWith(
                                CMD.goToSpecimenIntake(outtakeSubsystem)
                        ),
                        CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpA())
                        ),
                        CMD.clipSpecimen(outtakeSubsystem, 0.3),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnA())
                        /*
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.firstSpike()).setSpeed(0.8).alongWith(
                                CMD.sleep(600).andThen(
                                        CMD.extendIntake(intakeSubsystem, 0.7, 697)
                                )
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.firstHp(), true).setSpeed(1),
                        CMD.releaseSample(intakeSubsystem),
                        CMD.sleep(200),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.secondSpike(), true).setSpeed(0.8).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.7, 697)
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.secondHp()),
                        CMD.releaseSample(intakeSubsystem),
                        CMD.sleep(200),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.thirdSpike(), true).setSpeed(1).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.7, 680)
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.thirdHp()),
                        CMD.releaseSample(intakeSubsystem),

                        // ------------

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.startCycling()).alongWith(
                            CMD.retractIntake(intakeSubsystem)
                        ).alongWith(
                                CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpA())
                                )
                        ), CMD.clipSpecimen(outtakeSubsystem, 0.2),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnA()).alongWith(
                                CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpB())
                                )
                        ), CMD.clipSpecimen(outtakeSubsystem, 0.2),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnB()).alongWith(
                                CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpC())
                                )
                        ), CMD.clipSpecimen(outtakeSubsystem, 0.2),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnC()).alongWith(
                                CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpD())
                                )
                        ), CMD.clipSpecimen(outtakeSubsystem, 0.2),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnD())

                        */
                )
        );

    }
}
