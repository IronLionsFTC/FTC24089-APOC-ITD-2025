package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

import core.computerVision.Limelight;
import core.paths.SampleAutonomousV5;

public class SubToCvCached extends CommandBase {
    private Follower follower;
    private PathChain pathChain;

    private double speed;
    private boolean holdEnd;

    private Limelight.SampleState cached;

    public SubToCvCached(Follower follower, Limelight.SampleState cached) {
        this.follower = follower;
        this.cached = cached;
        this.holdEnd = true;
        this.speed = 0.8;
    }

    public SubToCvCached setSpeed(double speed) {
        this.speed = speed;
        this.follower.setMaxPower(speed);
        return this;
    }

    @Override
    public void initialize() {
        this.pathChain = SampleAutonomousV5.cachedSubToCv(cached);
        this.follower.breakFollowing();
        this.follower.setMaxPower(this.speed);
        this.follower.followPath(this.pathChain, this.holdEnd);
    }

    @Override
    public void execute() { this.follower.update(); }

    @Override
    public boolean isFinished() {
        return this.follower.getCurrentTValue() > 0.95;
    }
}
