package teleopmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.controls.Controls.Buttons;
import core.subsystems.Drivebase;
import core.subsystems.Intake;
import core.commands.CMD;

@TeleOp(name = "Teleop", group = "Competition")
public class Teleop extends CommandOpMode {

    private Drivebase drivebaseSubsystem;
    private Intake intakeSubsystem;
    private Buttons buttons = new Buttons(gamepad1, gamepad2);

    @Override
    public void initialize() {

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

                new ParallelCommandGroup(
                        CMD.setDriveVector(drivebaseSubsystem, buttons.driveX, buttons.driveY, buttons.yaw),
                        CMD.rotateIntakeClaw(intakeSubsystem, buttons.rotateClawRight, buttons.rotateClawLeft)
                )
        );
    }
}
