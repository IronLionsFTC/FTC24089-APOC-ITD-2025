package teleopmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.subsystems.Drivebase;
import core.subsystems.Intake;
import core.commands.CMD;

@TeleOp(name = "Teleop", group = "Competition")
public class Teleop extends CommandOpMode {

    private Drivebase drivebaseSubsystem;
    private Intake intakeSubsystem;

    @Override
    public void initialize() {

        // IMPORTANT - Register SUBSYSTEMS that implement periodic
        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);

        // Schedule the command based opmode
        schedule(
                CMD.sleepUntil(this::opModeIsActive),

                new ParallelCommandGroup(
                        CMD.setDriveVector(drivebaseSubsystem, 0, 0)
                )
        )
    }
}
