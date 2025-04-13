package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

import core.math.Vector;

public class SearchForever extends CommandBase {
    private Follower follower;
    private int state;
    public double speed;

    public SearchForever(Follower follower) {
        this.follower = follower;
        this.state = 0;
        this.speed = 0.4;
    }

    public void followVec(Vector rel) {
        // Calculate current position and rotation
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double r = follower.getPose().getHeading();

        double relativeX = rel.y * Math.cos(r) + rel.x * Math.sin(r);
        double relativeY = rel.x * Math.cos(r) + rel.y * Math.sin(r);

        double targetX = x + relativeX;
        double targetY = y - relativeY;

        PathBuilder builder = new PathBuilder();
        builder.addPath(
                new BezierLine(
                        new Point(x, y, Point.CARTESIAN),
                        new Point(targetX, targetY, Point.CARTESIAN)
                )
        ).setConstantHeadingInterpolation(r);

        follower.followPath(builder.build(), true);
    }

    public SearchForever setSpeed(double speed) {
        this.speed = speed;
        return this;
    }

    @Override
    public void initialize() {
        followVec(Vector.cartesian(5, 8));
        this.follower.setMaxPower(this.speed);
    }

    @Override
    public void execute() {
        if (follower.getCurrentTValue() > 0.95) {
            this.state += 1;
            if (this.state >= 4) this.state = 0;
            if (state == 0) {
                followVec(Vector.cartesian(-5, 9));
            } else if (this.state == 1) {
                followVec(Vector.cartesian(12, 0));
            } else if (this.state == 2) {
                followVec(Vector.cartesian(0, -9));
            } else {
                followVec(Vector.cartesian(-7, 0));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean i) {
        this.follower.setMaxPower(1);
    }
}