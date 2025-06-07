package autopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
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

@Autonomous ( name = "SAMPLE AUTONOMOUS V5" )
public class SampleAutonomousV5 extends CommandOpMode {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Follower follower;

    private Limelight limelight;
    private Limelight.SampleState buffer;

    private IndicatorLight light;

    private Command grabTransferDump(long delay) {
        return CMD.waitAndGrabSample(intakeSubsystem).andThen(
                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
        ).alongWith(
                CMD.sleep(delay).andThen(CMD.slamDunkSample(outtakeSubsystem))
        );
    }

    private Command extendAllAndMove(PathChain path, double r, double e, long delay, double speed) {
        return (CMD.followPath(follower, path).setSpeed(speed).alongWith(
                CMD.sleep(delay).andThen(CMD.raiseSlidesForSampleDump(outtakeSubsystem))
        )).alongWith(
                CMD.sleep(delay).andThen(CMD.extendIntake(intakeSubsystem, r, e))
        );
    }

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

                        CMD.followPath(follower, core.paths.SampleAutonomousV5.firstDumpAndPickup()).setSpeed(0.7).alongWith(
                                CMD.sleep(150).andThen(CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                        CMD.sleep(300).andThen(CMD.slamDunkSample(outtakeSubsystem))
                               ))
                        ).alongWith(
                                CMD.sleep(900).andThen(CMD.extendIntake(intakeSubsystem, 0.35, 700).andThen(
                                        CMD.waitAndGrabSample(intakeSubsystem).andThen(
                                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
                                        )
                                ))
                        ),

                        CMD.followPath(follower, core.paths.SampleAutonomousV5.secondDumpAndPickup()).alongWith(
                                CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                        CMD.sleep(300).andThen(CMD.slamDunkSample(outtakeSubsystem))
                                )
                        ).alongWith(
                                CMD.sleep(400).andThen(CMD.extendIntake(intakeSubsystem, 0.5, 546).andThen(
                                        CMD.waitAndGrabSample(intakeSubsystem).andThen(
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
                                CMD.sleep(300).andThen(CMD.extendIntake(intakeSubsystem, 0.55, 630))
                        ),
                        CMD.waitAndGrabSample(intakeSubsystem),

                        CMD.followPath(follower, core.paths.SampleAutonomousV5.lastDump()).alongWith(
                                CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem).andThen(
                                        CMD.raiseSlidesForSampleDump(outtakeSubsystem)
                                )
                        ),
                        CMD.sleep(300),
                        CMD.slamDunkSample(outtakeSubsystem),

                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, light),
                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, light),
                        CMD.subCycle(follower, intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, light)
                )
        );
    }
}
