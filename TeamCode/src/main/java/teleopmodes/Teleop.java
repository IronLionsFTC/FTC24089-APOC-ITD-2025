package teleopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.controls.Controls.Buttons;
import core.subsystems.Drivebase;
import core.subsystems.Intake;
import core.commands.CMD;

@TeleOp(name = "Teleop", group = "Competition")
public class Teleop extends CommandOpMode {

    private Drivebase drivebaseSubsystem;
    private Intake intakeSubsystem;
    private Buttons buttons;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.buttons = new Buttons(gamepad1, gamepad2);
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.drivebaseSubsystem = new Drivebase(hardwareMap, this.telemetry);

        // IMPORTANT - Register SUBSYSTEMS that implement periodic
        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);

        // Link buttons to commands
        buttons.intakeCycle.whenPressed(CMD.teleopIntakeCycle(intakeSubsystem));
        buttons.rotateRight.whenPressed(CMD.rotateCW(drivebaseSubsystem));
        buttons.rotateLeft.whenPressed(CMD.rotateCCW(drivebaseSubsystem));

        // Schedule the command based opmode
        schedule(
                CMD.sleepUntil(this::opModeIsActive),
                new RunCommand(telemetry::update),

                new ParallelCommandGroup(
                        CMD.setDriveVector(drivebaseSubsystem, buttons.driveX, buttons.driveY, buttons.yaw),
                        CMD.rotateIntakeClaw(intakeSubsystem, buttons.rotateClawRight, buttons.rotateClawLeft)
                )
        );
    }
}
