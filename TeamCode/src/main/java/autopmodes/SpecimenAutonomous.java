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

@Autonomous ( name = "SPECIMEN AUTONOMOUS" )
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
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.firstDump()).setSpeed(1).alongWith(
                                CMD.raiseSlidesForSpecimen(outtakeSubsystem)
                        ),
                        CMD.clipSpecimen(outtakeSubsystem, 0.3),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.firstSpike()).setSpeed(1).alongWith(
                                CMD.sleep(600).andThen(
                                        CMD.extendIntake(intakeSubsystem, 0.72, 0)
                                )
                        ),
                        CMD.sleep(500),
                        CMD.grabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.firstHp(), true).setSpeed(1),
                        CMD.releaseSample(intakeSubsystem),
                        CMD.sleep(200),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.secondSpike(), true).setSpeed(1).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.7, 0)
                        ),
                        CMD.sleep(500),
                        CMD.grabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.secondHp()),
                        CMD.releaseSample(intakeSubsystem),
                        CMD.sleep(200),

                        /*
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.thirdSpike(), true).setSpeed(1).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.65, 0)
                        ),
                        CMD.sleep(500),
                        CMD.grabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.thirdHp()),
                        CMD.releaseSample(intakeSubsystem),
                        CMD.sleep(200),
                        */

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.startCycling(), true),
                        CMD.setClawRotation(intakeSubsystem, 0.5),

                        CMD.sleep(1000),

                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpA(), true).setSpeed(1).alongWith(
                                CMD.sleep(500).andThen(CMD.raiseSlidesForSpecimen(outtakeSubsystem))
                        ),

                        CMD.clipSpecimen(outtakeSubsystem, 0.4),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnA(), true).setSpeed(1).alongWith(
                                CMD.sleep(800).andThen(CMD.extendIntake(intakeSubsystem))
                        ),

                        CMD.sleep(1000),

                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpB(), true).setSpeed(1).alongWith(
                                CMD.sleep(500).andThen(CMD.raiseSlidesForSpecimen(outtakeSubsystem))
                        ),
                        CMD.clipSpecimen(outtakeSubsystem, 0.5),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnB(), true).setSpeed(1).alongWith(
                                CMD.sleep(800).andThen(
                                        CMD.extendIntake(intakeSubsystem)
                                )
                        ),
                        CMD.sleep(1000),

                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpC(), true).setSpeed(1).alongWith(
                                CMD.sleep(500).andThen(CMD.raiseSlidesForSpecimen(outtakeSubsystem))
                        ),
                        CMD.clipSpecimen(outtakeSubsystem, 0.5),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnC(), true).setSpeed(1).alongWith(
                                CMD.sleep(800).andThen(
                                        CMD.extendIntake(intakeSubsystem)
                                )
                        ),
                        CMD.sleep(1000)

                        /*
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpD(), true).setSpeed(1).alongWith(
                                CMD.sleep(800).andThen(CMD.raiseSlidesForSpecimen(outtakeSubsystem))
                        ),
                        CMD.clipSpecimen(outtakeSubsystem, 0.6),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnD(), true).setSpeed(1).alongWith(
                                CMD.sleep(800).andThen(
                                        CMD.extendIntake(intakeSubsystem)
                                )
                        )
                        */
                )
        );

    }
}
