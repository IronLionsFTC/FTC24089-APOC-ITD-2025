package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

import core.computerVision.Limelight;
import core.math.Kinematics;
import core.paths.SampleAutonomousV2;
import core.paths.SampleAutonomousV5;

public class BasketToSubCached extends CommandBase {
    private Follower follower;
    private PathChain pathChain;

    private double speed;
    private boolean holdEnd;

    private Limelight.SampleState cached;

    public BasketToSubCached(Follower follower, Limelight.SampleState cached) {
        this.follower = follower;
        this.cached = cached;
        this.holdEnd = true;
        this.speed = 1;
    }

    public BasketToSubCached setSpeed(double speed) {
        this.speed = speed;
        this.follower.setMaxPower(speed);
        return this;
    }

    @Override
    public void initialize() {
        this.pathChain = SampleAutonomousV5.cachedBasketToSub(cached);
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

    @Override
    public void end(boolean wasInterrupted) {
        if (wasInterrupted) follower.holdPoint(follower.getPose());
        follower.setMaxPower(1);
    }
}
