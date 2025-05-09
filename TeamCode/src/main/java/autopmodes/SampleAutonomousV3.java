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

@Autonomous ( name = "SAMPLE AUTONOMOUS V3" )
public class SampleAutonomousV3 extends CommandOpMode {

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
        this.follower.setStartingPose(Vector.cartesian(-4, 0).pose(0));

        this.limelight = new Limelight(hardwareMap, Limelight.Targets.YellowOnly);
        this.buffer = new Limelight.SampleState();

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.light(light, 0.3),

                        CMD.followPath(follower, core.paths.SampleAutonomousV2.firstDumpAndPickup()).setSpeed(1).alongWith(
                            CMD.sleep(800).andThen(CMD.raiseSlidesForSampleDump(outtakeSubsystem))
                        ),

                        CMD.sleep(200).andThen(
                            CMD.slamDunkSample(outtakeSubsystem).andThen(
                                CMD.sleep(300)
                            )
                        ).alongWith(
                            CMD.extendIntake(intakeSubsystem).andThen(CMD.grabSample(intakeSubsystem))
                        ),

                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.secondDumpAndPickup()).alongWith(
                                        CMD.extendIntake(intakeSubsystem)
                                )
                        ),

                        CMD.sleep(200),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.sleep(300),

                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).alongWith(
                                CMD.extendIntake(intakeSubsystem)
                        ),

                        CMD.sleep(200),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.sleep(200),

                        CMD.followPath(follower, core.paths.SampleAutonomousV2.thirdDumpAndPickup()),

                        CMD.sleep(300),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                        ).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.lastDump()).setSpeed(1)
                        ),

                        CMD.sleep(200),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.raiseLimelight(limelight),

                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, light),
                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, light),
                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, light),
                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, light),
                        CMD.hideLimelight(limelight)
                )
        );
    }
}
