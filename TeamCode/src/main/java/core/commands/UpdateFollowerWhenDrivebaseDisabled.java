package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import core.subsystems.Drivebase;

public class UpdateFollowerWhenDrivebaseDisabled extends CommandBase {
    private Drivebase drivebase;
    private Follower follower;

    public UpdateFollowerWhenDrivebaseDisabled(Drivebase drivebase, Follower follower) {
        this.drivebase = drivebase;
        this.follower = follower;
    }

    @Override
    public void execute() {
        if (!this.drivebase.active()) follower.update();
    }
}
