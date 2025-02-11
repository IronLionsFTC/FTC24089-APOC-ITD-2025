package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.subsystems.Drivebase;

public class SetDriveVector extends CommandBase {
    private final Drivebase drivebaseSubsystem;

    public SetDriveVector(Drivebase drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        addRequirements(drivebaseSubsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
