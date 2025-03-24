package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import core.subsystems.Drivebase;

public class ConstantlyUpdateFollower extends CommandBase {
    private Follower follower;
    private Drivebase drivebase;

    public ConstantlyUpdateFollower(Follower follower, Drivebase drivebase) {
        this.follower = follower;
        this.drivebase = drivebase;
    }

    @Override
    public void execute() {
        if (!this.drivebase.active()) follower.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
