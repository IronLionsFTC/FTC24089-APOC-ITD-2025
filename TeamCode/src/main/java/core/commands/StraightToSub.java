package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

import core.computerVision.Limelight;
import core.paths.SampleAutonomousV5;

public class StraightToSub extends CommandBase {
    private Follower follower;
    private Limelight.SampleState cached;
    private PathChain path;

    public StraightToSub(Follower follower, Limelight.SampleState cached) {
        this.follower = follower;
        this.cached = cached;
    }

    @Override
    public void initialize() {
        this.path = SampleAutonomousV5.testBasketToSub(this.cached);
        this.follower.breakFollowing();
        this.follower.followPath(this.path, true);
        this.follower.setMaxPower(1.0);
    }

    @Override
    public void execute() {
        this.follower.update();
    }

    @Override
    public boolean isFinished() {
        return this.follower.getCurrentTValue() > 0.95;
    }
}
