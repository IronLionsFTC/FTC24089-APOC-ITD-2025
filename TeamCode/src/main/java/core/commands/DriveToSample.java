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
        this.follower.setMaxPower(0.3);

        // Calculate current position and rotation
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double r = follower.getPose().getHeading();

        double tx = -0.8 * buffer.center.x;
        double ty = 4 - 0.9 * buffer.center.y;

        double relativeX = ty * Math.cos(r) + tx * Math.sin(r);
        double relativeY = tx * Math.cos(r) + ty * Math.sin(r);

        double targetX = x + relativeX;
        double targetY = y - relativeY;

        // End current path if applicable then path to new location
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
    public void execute() { if (this.wasValid) follower.update(); }

    @Override
    public boolean isFinished() {
        return follower.getCurrentTValue() > 0.95 || !this.wasValid;
    }

    @Override
    public void end(boolean i) {
        this.follower.setMaxPower(1);
        buffer.center = Vector.cartesian(0, 0);
    }
}
