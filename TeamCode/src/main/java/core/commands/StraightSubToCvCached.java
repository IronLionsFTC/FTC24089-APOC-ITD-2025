package core.commands;

import static core.paths.SampleAutonomousV5.point;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import core.computerVision.Limelight;
import core.paths.SampleAutonomousV5;

public class StraightSubToCvCached extends CommandBase {
    private Follower follower;
    private PathChain pathChain;

    private double speed;
    private boolean holdEnd;

    private Limelight.SampleState cached;

    public StraightSubToCvCached(Follower follower, Limelight.SampleState cached) {
        this.follower = follower;
        this.cached = cached;
        this.holdEnd = true;
        this.speed = 1;
    }

    public StraightSubToCvCached setSpeed(double speed) {
        this.speed = speed;
        this.follower.setMaxPower(speed);
        return this;
    }

    @Override
    public void initialize() {
        double x = follower.getPose().getY();
        double y = follower.getPose().getX();
        Point current = point(x, y);
        this.pathChain = SampleAutonomousV5.cachedStraightSubToCv(current, cached);
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
