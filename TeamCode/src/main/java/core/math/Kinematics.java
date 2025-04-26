package core.math;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import core.computerVision.Limelight;

public class Kinematics {
    public RobotPosition absoluteRobotTarget;
    public double absoluteSlidePosition;
    public double absoluteClawRotation;

    public Kinematics(Limelight.SampleState buffer) {

        //                              Normal  hardstop pos             scale to axon   limelight angle
        double altitude = Math.toRadians(90 - ((75 - buffer.intakeTilt * 350) - buffer.center.y));
        double forward = (Math.tan(altitude) * 25) / 2.54;

        double slidePosition = buffer.slidePosition;
        double newSlidePosition = slidePosition + forward * 20;

        double ty = 0;

        if (newSlidePosition > 400) {
            double error = newSlidePosition - 400;
            newSlidePosition = 400;
            ty = error / 20;
        }

        double tx = Math.tan(Math.toRadians(buffer.center.x)) * forward * 4;

        this.absoluteRobotTarget = RobotPosition.relativePosition(
                new RobotPosition(
                        buffer.robotPosition,
                        buffer.robotRotation
                ),
                Vector.cartesian(
                        tx, ty
                )
        );
        this.absoluteSlidePosition = newSlidePosition;
        this.absoluteClawRotation = buffer.angle;
    }

    public PathChain instantPath(Follower follower) {
        Pose pose = follower.getPose();
        double cx = pose.getX();
        double cy = pose.getY();

        PathBuilder builder = new PathBuilder();
        builder.addPath(
                new BezierLine(
                        new Point(cx, cy, Point.CARTESIAN),
                        new Point(
                                this.absoluteRobotTarget.position.x,
                                this.absoluteRobotTarget.position.y,
                                Point.CARTESIAN
                        )
                )
        ).setConstantHeadingInterpolation(
                this.absoluteRobotTarget.rotation
        );

        return builder.build();
    }
}
