package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

import core.math.Vector;

public class MoveAbsolute extends CommandBase {
    private Follower follower;
    private Vector position;
    private boolean holdEnd;

    public MoveAbsolute(Follower follower, Vector position, boolean holdEnd) {
        this.follower = follower;
        this.position = position;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        this.follower.breakFollowing();
        PathBuilder builder = new PathBuilder();
        builder.addPath(
                new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), 1),
                        new Point(position.y, -position.x, 1)
                )
        ).setConstantHeadingInterpolation(follower.getPose().getHeading());
        this.follower.breakFollowing();
        this.follower.followPath(builder.build(), this.holdEnd);
    }

    @Override
    public boolean isFinished() {
        return this.follower.getCurrentTValue() > 0.95;
    }
}
