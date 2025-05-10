package core.math;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import core.computerVision.Limelight;

public class Kinematics {

    @Config
    public static class LimelightInformation {
        public static double limelightHeight = 13.7;
        public static double limelightAngle = 65;
        public static double forwardOffset = 0.5;

        public static double forwardScalarForLateral = 0.01143;
        public static double forwardOffsetForLateral = 0.2661;

        public static double constantXOffset = 4.9;
    }
    public RobotPosition absoluteRobotTarget;
    public double absoluteSlidePosition;
    public double absoluteClawRotation;

    public Kinematics(Limelight.SampleState buffer) {

        double forwards = LimelightInformation.limelightHeight
                / (Math.tan(Math.toRadians(LimelightInformation.limelightAngle - buffer.center.y)))
                + LimelightInformation.forwardOffset;

        double lateral = (LimelightInformation.forwardScalarForLateral * forwards + LimelightInformation.forwardOffsetForLateral) * 0.8 // Derived from experiments not maths
                * buffer.center.x + LimelightInformation.constantXOffset;

        double newSlidePosition = (forwards + 3) * 25;
        double ty = 0;

        if (newSlidePosition > 400) {
            double error = newSlidePosition - 400;
            newSlidePosition = 400;
            ty = error / 20;
        }

        this.absoluteRobotTarget = RobotPosition.relativePosition(
                new RobotPosition(
                        buffer.robotPosition,
                        buffer.robotRotation
                ),
                Vector.cartesian(
                        lateral * -1, ty
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
