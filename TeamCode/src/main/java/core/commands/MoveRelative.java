package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

import core.math.Vector;

public class MoveRelative extends CommandBase {
    private Follower follower;
    private Vector position;
    private boolean holdEnd;
    private double maxSpeed;

    public MoveRelative(Follower follower, Vector position, boolean holdEnd, double maxSpeed) {
        this.follower = follower;
        this.position = position;
        this.holdEnd = holdEnd;
        this.maxSpeed = maxSpeed;
    }

    public MoveRelative(Follower follower, Vector position, boolean holdEnd) {
        this.follower = follower;
        this.position = position;
        this.holdEnd = holdEnd;
        this.maxSpeed = 0.4;
    }

    @Override
    public void initialize() {
        this.follower.setMaxPower(maxSpeed);
        // Calculate current position and rotation
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double r = follower.getPose().getHeading();

        double relativeX = position.y * Math.cos(r) + position.x * Math.sin(r);
        double relativeY = position.x * Math.cos(r) + position.y * Math.sin(r);

        double targetX = x + relativeX;
        double targetY = y - relativeY;

        // End current path if applicable then path to new location
        follower.breakFollowing();

        PathBuilder builder = new PathBuilder();
        builder.addPath(
                new BezierLine(
                        new Point(x, y, Point.CARTESIAN),
                        new Point(targetX, targetY, Point.CARTESIAN)
                )
        ).setConstantHeadingInterpolation(r);
        follower.followPath(builder.build(), this.holdEnd);
    }

    @Override
    public boolean isFinished() {
        return this.follower.getCurrentTValue() > 0.95;
    }

    @Override
    public void end(boolean interrupted) {
        follower.setMaxPower(1);
    }
}
