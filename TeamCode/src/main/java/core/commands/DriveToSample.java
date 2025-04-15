package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

import core.computerVision.Limelight;
import core.math.Vector;

public class DriveToSample extends CommandBase {
    private Follower follower;
    private Limelight.SampleState buffer;
    private boolean wasValid;

    public DriveToSample(Follower follower, Limelight.SampleState buffer) {
        this.follower = follower;
        this.buffer = buffer;
        this.wasValid = false;
    }

    @Override
    public void initialize() {

        this.wasValid = true;
        this.follower.setMaxPower(0.4);

        // Calculate current position and rotation
        double x = buffer.robotPosition.x;
        double y = buffer.robotPosition.y;
        double r = buffer.robotRotation;

        double cx = follower.getPose().getX();
        double cy = follower.getPose().getY();

        double tx = 1 - 0.8 * buffer.center.x;
        double ty = 2.7 - buffer.center.y;

        double relativeX = ty * Math.cos(r) + tx * Math.cos(r - Math.toRadians(90));
        double relativeY = ty * Math.sin(r) + tx * Math.sin(r - Math.toRadians(90));

        double targetX = x + relativeX;
        double targetY = y + relativeY;

        // End current path if applicable then path to new location
        PathBuilder builder = new PathBuilder();
        builder.addPath(
                new BezierLine(
                        new Point(cx, cy, Point.CARTESIAN),
                        new Point(targetX, targetY, Point.CARTESIAN)
                )
        ).setConstantHeadingInterpolation(r);
        follower.followPath(builder.build(), true);
    }

    @Override
    public void execute() { if (this.wasValid) follower.update(); }

    @Override
    public boolean isFinished() {
        return follower.getCurrentTValue() > 0.95 || !this.wasValid;
    }

    @Override
    public void end(boolean i) {
        this.follower.setMaxPower(1);
        buffer.center = Vector.cartesian(0, 0);
        buffer.robotPosition = Vector.cartesian(0, 0);
        buffer.robotRotation = 0;
    }
}
