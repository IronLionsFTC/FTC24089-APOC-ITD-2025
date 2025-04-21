package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import core.math.Vector;
import core.subsystems.Intake;

public class SearchForeverUseSlides extends CommandBase {
    private Follower follower;
    private Intake intakeSubsystem;
    private int state;
    public double speed;

    private Timer timer;

    public SearchForeverUseSlides(Follower follower, Intake intakeSubsystem) {
        this.follower = follower;
        this.intakeSubsystem = intakeSubsystem;
        this.state = 0;
        this.speed = 0.4;
        this.timer = new Timer();
    }

    public void followVec(Vector rel) {
        // Calculate current position and rotation
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double r = follower.getPose().getHeading();

        double relativeX = rel.y * Math.cos(r) + rel.x * Math.cos(r - Math.toRadians(90));
        double relativeY = rel.y * Math.sin(r) + rel.x * Math.sin(r - Math.toRadians(90));

        double targetX = x + relativeX;
        double targetY = y + relativeY;

        PathBuilder builder = new PathBuilder();
        builder.addPath(
                new BezierLine(
                        new Point(x, y, Point.CARTESIAN),
                        new Point(targetX, targetY, Point.CARTESIAN)
                )
        ).setConstantHeadingInterpolation(r);

        follower.followPath(builder.build(), true);
    }

    public SearchForeverUseSlides setSpeed(double speed) {
        this.speed = speed;
        return this;
    }

    @Override
    public void initialize() {
        this.state = 0;
        this.followVec(Vector.cartesian(0, 8));
        this.follower.setMaxPower(this.speed);
        this.intakeSubsystem.setOffset(0.3);
    }

    @Override
    public void execute() {
        this.intakeSubsystem.setOffset((1 - Math.cos(timer.getElapsedTimeSeconds() * 2)) * 0.125 + 0.25);
        if (follower.getCurrentTValue() > 0.95) {
            this.state += 1;
            if (this.state > 1) this.state = 0;
            if (state == 0) {
                followVec(Vector.cartesian(-8, 0));
            } else if (this.state == 1) {
                followVec(Vector.cartesian(8, 0));
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