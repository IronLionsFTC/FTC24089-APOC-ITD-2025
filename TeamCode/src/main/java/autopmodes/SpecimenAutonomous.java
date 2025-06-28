package autopmodes;

import android.widget.GridLayout;

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
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.firstDump()).setSpeed(1.0).alongWith(
                                CMD.raiseSlidesForSpecimen(outtakeSubsystem)
                        ),
                        CMD.clipSpecimen(outtakeSubsystem, 0.3).alongWith(
                                CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false).andThen(
                                        CMD.driveToSampleUseSlidesSpec(follower, intakeSubsystem, buffer, telemetry).alongWith(
                                                CMD.alignClaw(intakeSubsystem, buffer)
                                        ).andThen(
                                                CMD.waitAndGrabSample(intakeSubsystem)
                                        )
                                )
                        ),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnCV()).setSpeed(0.6).alongWith(
                                CMD.retractIntakeAndTransferHalf(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.goToSpecimenIntake(outtakeSubsystem)
                                )
                        ),

                        CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goCV()).setSpeed(1.0).alongWith(
                                CMD.raiseSlidesForSpecimen(outtakeSubsystem)
                        ),

                        CMD.clipSpecimen(outtakeSubsystem, 0.3),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.firstSpike()).setSpeed(1.0).alongWith(
                                CMD.sleep(800).andThen(CMD.extendIntake(intakeSubsystem, 0.7, 697))
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.firstHp(), false).setSpeed(1),
                        CMD.releaseSample(intakeSubsystem),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.secondSpike(), true).setSpeed(1.0).alongWith(
                                CMD.sleep(300).andThen(CMD.extendIntake(intakeSubsystem, 0.7, 697))
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.secondHp()).setSpeed(1.0),
                        CMD.releaseSample(intakeSubsystem),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.thirdSpike(), true).setSpeed(1.0).alongWith(
                                CMD.sleep(300).andThen(CMD.extendIntake(intakeSubsystem, 0.7, 600))
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),
                        CMD.followPath(follower, core.paths.SpecimenAutonomous.thirdHp(), false).setSpeed(1.0),
                        CMD.retractIntake(intakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SpecimenAutonomous.startCycling())
                        ).alongWith(
                                CMD.goToSpecimenIntake(outtakeSubsystem)
                        ),

                        // ------------

                        CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpA())
                        ), CMD.clipSpecimen(outtakeSubsystem, 0.3),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnA()).alongWith(
                                CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpB())
                                )
                        ), CMD.clipSpecimen(outtakeSubsystem, 0.3),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnB()).alongWith(
                                CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpC())
                                )
                        ), CMD.clipSpecimen(outtakeSubsystem, 0.3),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnC()).alongWith(
                                CMD.specimenIntakeBySensor(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.followPath(follower, core.paths.SpecimenAutonomous.goDumpD())
                                )
                        ), CMD.clipSpecimen(outtakeSubsystem, 0.3),

                        CMD.followPath(follower, core.paths.SpecimenAutonomous.returnD())
                )
        );

    }
}
