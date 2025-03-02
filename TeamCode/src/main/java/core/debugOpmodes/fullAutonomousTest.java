package core.debugOpmodes;

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
import core.math.Vector;
import core.subsystems.Intake;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Full Autonomous Test")
public class fullAutonomousTest extends CommandOpMode {
    private Follower follower;
    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry);

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(Vector.cartesian(0, 0).pose(0));

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        CMD.sleepUntil(this::opModeIsActive),
                        CMD.sleep(4000),

                        // Grab the first sample
                        CMD.extendIntake(intakeSubsystem),
                        CMD.grabSample(intakeSubsystem),
                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),

                        // Outtake the first sample
                        CMD.moveAbsolute(follower, Vector.cartesian(10, 0), true).andThen(CMD.sleep(500)).alongWith(CMD.extendIntake(intakeSubsystem)).andThen(CMD.grabSample(intakeSubsystem))
                                .alongWith(
                                        CMD.raiseSlidesForSampleDump(outtakeSubsystem).andThen(
                                                CMD.slamDunkSample(outtakeSubsystem).andThen(
                                                        CMD.waitUntilOuttakeDown(outtakeSubsystem)
                                                )
                                        )
                                ),

                        CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem),
                        CMD.raiseSlidesForSampleDump(outtakeSubsystem),
                        CMD.slamDunkSample(outtakeSubsystem),
                        CMD.waitUntilOuttakeDown(outtakeSubsystem).alongWith(
                                CMD.moveAbsolute(follower, Vector.cartesian(0, 0), true)
                        )
                )
        );
    }
}
