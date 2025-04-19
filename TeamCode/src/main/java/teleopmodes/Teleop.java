package teleopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.computerVision.Limelight;
import core.controls.Controls.Buttons;
import core.math.Vector;
import core.subsystems.Drivebase;
import core.subsystems.Intake;
import core.commands.CMD;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "Teleop", group = "Competition")
public class Teleop extends CommandOpMode {

    private Drivebase drivebaseSubsystem;
    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Buttons buttons;
    private Follower follower;

    private Limelight limelight;
    private Limelight.SampleState sampleState;

    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(Vector.cartesian(0, 0).pose(0));

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.buttons = new Buttons(gamepad1, gamepad2);
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry);
        this.drivebaseSubsystem = new Drivebase(hardwareMap, this.telemetry);

        this.limelight = new Limelight(hardwareMap, Limelight.Targets.RedAndYellow);
        this.sampleState = new Limelight.SampleState();

        // IMPORTANT - Register SUBSYSTEMS that implement periodic
        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);

        // Link buttons to commands
        buttons.intakeCycle.whenPressed(CMD.teleopIntakeCycle(intakeSubsystem));
        buttons.outtakeCycle.whenPressed(CMD.teleopOuttakeCycle(outtakeSubsystem));

        buttons.rotateRight.whenPressed(CMD.rotateCW(drivebaseSubsystem));
        buttons.rotateLeft.whenPressed(CMD.rotateCCW(drivebaseSubsystem));

        buttons.override.whenPressed(CMD.teleopOverride(intakeSubsystem, outtakeSubsystem));

        buttons.useCV.whenPressed(
            CMD.resetCV(sampleState).andThen(
                CMD.scanForSample(limelight, sampleState, telemetry, follower, intakeSubsystem, false).andThen(
                    CMD.alignClaw(intakeSubsystem, sampleState)
                )
            )
        );

        // Schedule the command based opmode
        schedule(
                CMD.sleepUntil(this::opModeIsActive),
                new RunCommand(telemetry::update),
                new ParallelCommandGroup(
                        CMD.constantlyUpdateFollower(follower, drivebaseSubsystem),
                        CMD.setDriveVector(drivebaseSubsystem, intakeSubsystem, buttons.driveX, buttons.driveY, buttons.yaw),
                        CMD.rotateIntakeClaw(intakeSubsystem, buttons.rotateClawRight, buttons.rotateClawLeft),
                        CMD.teleopAutomaticTransfer(intakeSubsystem, outtakeSubsystem)
                )
        );
    }
}
