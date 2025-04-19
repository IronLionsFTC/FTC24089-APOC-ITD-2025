package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

import core.computerVision.Limelight;
import core.math.Vector;
import core.subsystems.Intake;

public class DriveToSampleUseSlides extends CommandBase {
    private Follower follower;
    private Intake intakeSubsystem;
    private Limelight.SampleState buffer;
    private boolean wasValid;

    public DriveToSampleUseSlides(Follower follower, Intake intakeSubsystem, Limelight.SampleState buffer) {
        this.follower = follower;
        this.intakeSubsystem = intakeSubsystem;
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

        double theta = Math.toRadians(90 - ((75- intakeSubsystem.getTilt() * 30) - buffer.center.y));
        double cm = Math.tan(theta) * 25; // height
        double inches = cm / 2.54;

        double slidePosition = buffer.slidePosition;
        double slideMovement = (-inches - 2.7) * 0.03;
        double slideTarget   = slidePosition + slideMovement;

        double newTy = 0;

        if (slideTarget > 0.6) {
            double error = slideTarget - 0.6;
            slideMovement -= error;
            newTy = error / 0.03;
        } else if (slideTarget < 0) {
            double error = -slideTarget;
            slideMovement += error;
            newTy = -error / 0.03;
        }

        double xInches = Math.tan(Math.toRadians(buffer.center.x)) * inches;

        double tx = xInches;
        double ty = newTy;

        intakeSubsystem.setOffset(buffer.slideOffset + slideMovement);

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
