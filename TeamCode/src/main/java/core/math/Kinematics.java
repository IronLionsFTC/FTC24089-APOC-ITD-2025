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

        public static double forwardScalarForLateral = 0.01;
        public static double forwardOffsetForLateral = 0.2661;

        public static double constantXOffset = 4.9;

        public static double a = 24.32983;
        public static double b = 0.0310553;
        public static double c = -8.773041;
        public static double e = Math.E;
    }
    public RobotPosition absoluteRobotTarget;
    public double absoluteSlidePosition;
    public double absoluteClawRotation;

    public Kinematics(Limelight.SampleState buffer) {
        double slideExtension = 319.77939 * Math.pow(Math.E, 0.0310912 * buffer.center.y) - 3.62055;
        //slideExtension *= 0.95;
        double lateralGradient = -0.000871067 * slideExtension - 0.474881;
        double lateralIntercept = -10.45;

        double lateralCM = (lateralGradient * buffer.center.x + lateralIntercept) * 0.83;
        double ty = 0;

        if (slideExtension > 700) {
            double error = slideExtension - 700;
            slideExtension = 700;
            ty = error / 20;
        }

        this.absoluteRobotTarget = RobotPosition.relativePosition(
                new RobotPosition(
                        buffer.robotPosition,
                        buffer.robotRotation
                ),
                Vector.cartesian(
                        lateralCM / 2.54, ty
                )
        );
        this.absoluteSlidePosition = slideExtension - 25;
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
