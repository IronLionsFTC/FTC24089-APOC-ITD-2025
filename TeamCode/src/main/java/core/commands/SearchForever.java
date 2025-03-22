package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

import core.math.Vector;

public class SearchForever extends CommandBase {
    private Follower follower;
    private boolean forward;

    public SearchForever(Follower follower) {
        this.follower = follower;
        this.forward = true;
    }

    public void followVec(Vector rel) {
        this.follower.setMaxPower(0.3);
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

    @Override
    public void initialize() {
        followVec(Vector.cartesian(5, 8));
    }

    @Override
    public void execute() {
        if (follower.getCurrentTValue() > 0.95) {
            this.forward = !this.forward;
            if (this.forward) {
                followVec(Vector.cartesian(5, 8));
            } else {
                followVec(Vector.cartesian(-5, -8));
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