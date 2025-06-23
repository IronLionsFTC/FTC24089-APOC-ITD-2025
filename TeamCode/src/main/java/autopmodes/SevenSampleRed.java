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

@Autonomous ( name = "- 7 Sample [RED] -" )
public class SevenSampleRed extends CommandOpMode {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Follower follower;

    private Limelight limelight;
    private Limelight.SampleState buffer;
    private Limelight.SampleState cache;

    private IndicatorLight light;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry, this.intakeSubsystem::forceDown);
        this.light = new IndicatorLight(hardwareMap, "light");

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);

        Constants.setConstants(FConstants.class, LConstants.class);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(Vector.cartesian(-2, 0).pose(0));

        this.limelight = new Limelight(hardwareMap, Limelight.Targets.RedAndYellow);
        this.buffer = new Limelight.SampleState();
        this.cache = new Limelight.SampleState();

        limelight.raise();

        schedule(
                new RunCommand(follower::update),
                new RunCommand(telemetry::update),
                new SequentialCommandGroup(
                        CMD.waitForStartWithPreloadWarning(light, intakeSubsystem, this::opModeIsActive),

                        CMD.followPath(follower, core.paths.SampleAutonomousV5.firstDumpAndPickup()).setSpeed(0.7).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                    CMD.waitForProgress(follower, 0.75).andThen(CMD.slamDunkSample(outtakeSubsystem))
                                )
                        ).alongWith(
                                CMD.sleep(800).andThen(CMD.extendIntake(intakeSubsystem, 0.35, 700).andThen(
                                        CMD.shortWaitAndGrabSample(intakeSubsystem).andThen(
                                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
                                        )
                                ))
                        ),

                        CMD.followPath(follower, core.paths.SampleAutonomousV5.secondDumpAndPickup()).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                        CMD.sleep(300).andThen(CMD.slamDunkSample(outtakeSubsystem))
                                )
                        ).alongWith(
                                CMD.sleep(200).andThen(CMD.extendIntake(intakeSubsystem, 0.5, 600).andThen(
                                        CMD.shortWaitAndGrabSample(intakeSubsystem).andThen(
                                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
                                        )
                                ))
                        ),

                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).alongWith(
                                CMD.followPath(follower, core.paths.SampleAutonomousV5.secondDumpAndPickup())
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),

                        CMD.followPath(follower, core.paths.SampleAutonomousV5.thirdDumpAndPickup()).alongWith(
                                CMD.sleep(300).andThen(CMD.extendIntake(intakeSubsystem, 0.55, 655))
                        ),
                        CMD.sleep(300).andThen(CMD.grabSample(intakeSubsystem)),

                        CMD.followPath(follower, core.paths.SampleAutonomousV5.lastDump()).alongWith(
                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                                )
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),

                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, cache, telemetry, light),
                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, cache, telemetry, light),
                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, cache, telemetry, light)
                )
        );
    }
}
