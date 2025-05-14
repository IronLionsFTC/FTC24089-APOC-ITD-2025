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

@Autonomous ( name = "SAMPLE AUTONOMOUS V4" )
public class SampleAutonomousV4 extends CommandOpMode {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Follower follower;

    private Limelight limelight;
    private Limelight.SampleState buffer;

    private IndicatorLight light;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry);
        this.light = new IndicatorLight(hardwareMap, "light");

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);

        Constants.setConstants(FConstants.class, LConstants.class);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(Vector.cartesian(-4, 0).pose(0));

        this.limelight = new Limelight(hardwareMap, Limelight.Targets.YellowOnly);
        this.buffer = new Limelight.SampleState();

        schedule(
                new RunCommand(follower::update),
                new RunCommand(telemetry::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),

                        CMD.followPath(follower, core.paths.SampleAutonomousV2.firstDumpAndPickup()).setSpeed(1).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.4, 520).andThen(
                                    CMD.sleep(300)
                                ).andThen(
                                        CMD.grabSample(intakeSubsystem)
                                ).andThen(
                                        CMD.sleep(300)
                                ).andThen(
                                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
                                )
                        ).alongWith(
                                CMD.sleep(400).andThen(
                                        CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                                ).andThen(
                                        CMD.sleep(300)
                                ).andThen(
                                        CMD.slamDunkSample(outtakeSubsystem)
                                ).andThen(
                                        CMD.sleep(300)
                                )
                        ),

                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                CMD.sleep(200).andThen(
                                        CMD.slamDunkSample(outtakeSubsystem)
                                ).andThen(
                                        CMD.sleep(300)
                                )
                        ).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.secondDumpAndPickup()).alongWith(
                                        CMD.extendIntake(intakeSubsystem, 0.5, 520).andThen(
                                                CMD.sleep(300)
                                        ).andThen(
                                                CMD.grabSample(intakeSubsystem)
                                        ).andThen(
                                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
                                        )
                                )
                        ),


                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                CMD.sleep(200),
                                CMD.slamDunkSample(outtakeSubsystem),
                                CMD.followPath(follower, core.paths.SampleAutonomousV2.thirdDumpAndPickup())
                        ).alongWith(
                                CMD.extendIntake(intakeSubsystem, 0.6, 440)
                        ),

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
                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, light)
                )
        );
    }
}
