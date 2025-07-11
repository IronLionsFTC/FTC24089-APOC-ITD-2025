package core.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierCurveCoefficients;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import core.computerVision.Limelight;
import core.math.Kinematics;

public class SampleAutonomousV5 {
    public static Point point(double x, double y) {
        return new Point(y, x, Point.CARTESIAN);
    }

    public static PathChain simpleLine(Point a, Point b, double h) {
        Path path = new Path(
                new BezierLine(a, b)
        );
        path.setConstantHeadingInterpolation(Math.toRadians(h));
        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }

    public static PathChain simpleTangentLine(Point a, Point b) {
        Path path = new Path(
                new BezierLine(a, b)
        );
        path.setTangentHeadingInterpolation();
        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }

    public static PathChain constantCurve(Point a, Point c, Point b, double h) {
        Path path = new Path(
                new BezierCurve(a, c, b)
        );
        path.setConstantHeadingInterpolation(Math.toRadians(h));
        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }

    public static PathChain simpleCurve(Point a, Point c, Point b) {
        Path path = new Path(
                new BezierCurve(a, c, b)
        );
        path.setTangentHeadingInterpolation();
        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }

    public static PathChain simpleReverseCurve(Point a, Point c, Point c2, Point b) {
        Path path = new Path(
                new BezierCurve(a, c, c2, b)
        );
        path.setTangentHeadingInterpolation();
        path.setReversed(true);
        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }

    public static Point start = point(0, 0);
    public static Point stageOne = point(17, 7.6);
    public static Point lastDump = point(18.5, 5.5);
    public static Point cvDump = point(17, 13.5);
    public static Point cvDumpFurther = point(13, 11.5);
    public static Point stageTwo = point(20.5, 9);
    public static Point stageTwoDump = point(17.5, 7);
    public static Point stageThree = point(20.69, 9);
    public static Point submersible = point(-10, 53);
    public static Point basketToSubControl = point(15, 50);
    public static Point subToBasketControl = point(11, 17);
    public static Point cvStart = point(-13, 53);
    public static Point fourthDump = point(17.5, 7);
    public static Point testCVDumpStart = point(-6, 54);
    public static Point testCVDump = point(19, 9);

    public static PathChain testCV(Follower follower) {
        Path currentToCV = currentToCV(follower);
        Path CVToBasket = simpleLine(testCVDumpStart, testCVDump, -30).getPath(0);

        PathBuilder builder = new PathBuilder();
        currentToCV.setConstantHeadingInterpolation(Math.toRadians(-30));
        CVToBasket.setConstantHeadingInterpolation(Math.toRadians(-30));
        builder.addPath(currentToCV);
        builder.addPath(CVToBasket);

        return builder.build();
    }

    public static PathChain testBasketToSub(Limelight.SampleState cached) {
        double x = 53;
        if (cached.angle != 0 && cached.angle != 90) {
            Kinematics kinematics = new Kinematics(cached);
            if (kinematics.absoluteRobotTarget.position.x >= 50) {
                x = kinematics.absoluteRobotTarget.position.x;
            }
        }
        Point cache = point(-5, x);

        Path straightThere = new Path(
                new BezierLine(
                        testCVDump, cache
                )
        );

        straightThere.setTangentHeadingInterpolation();

        Path toSample = new Path(
                new BezierLine(
                    cache,
                    point(-13, x)
                )
        );

        toSample.setConstantHeadingInterpolation(Math.toRadians(-90));

        PathBuilder chain = new PathBuilder();
        chain.addPath(straightThere);
        chain.addPath(toSample);
        return chain.build();
    }

    public static Path currentToCV(Follower follower) {
        Pose current = follower.getPose();
        Point currentPoint = point(current.getY(), current.getX());
        return simpleLine(currentPoint, point(-6, current.getX()), -30).getPath(0);
    }

    public static PathChain testFirstDump() {
        return simpleLine(start, lastDump, -20);
    }
    public static PathChain testFirstPickup() {
        return simpleLine(lastDump, stageOne, -17);
    }
    public static PathChain secondDumpAndPickup() {
        return simpleLine(stageOne, stageTwo, -7.5);
    }

    public static PathChain dumpFromFirst() {
        return simpleLine(stageOne, stageTwoDump, -40);
    }

    public static PathChain testSecond() {
        return simpleLine(stageTwoDump, stageTwo, -6.5);
    }

    public static PathChain dumpFromSecond() {
        return simpleLine(stageTwo, stageTwoDump, -40);
    }
    public static PathChain thirdDumpAndPickup() {
        return simpleLine(stageTwo, stageThree, 20);
    }
    public static PathChain lastDump() {
        return simpleLine(stageThree, fourthDump, -40);
    }
    public static PathChain basketToSub() {
        return simpleCurve(stageOne, basketToSubControl, submersible);
    }

    public static PathChain cachedBasketToSub(Limelight.SampleState cached) {
        if (cached.angle != 0 && cached.angle != 90) {
            Kinematics kinematics = new Kinematics(cached);
            if (kinematics.absoluteRobotTarget.position.x >= 50) {
                return simpleCurve(stageOne, basketToSubControl,
                    point(-10, kinematics.absoluteRobotTarget.position.x)
                );
            } else {
                return basketToSub();
            }
        } else {
            return basketToSub();
        }
    }

    public static PathChain cachedStraightBasketToSub(Point current, Limelight.SampleState cached) {
        double endX = 0;
        double endY;

        if (cached.angle != 0 && cached.angle != 90) {
            Kinematics kinematics = new Kinematics(cached);
            if (kinematics.absoluteRobotTarget.position.x >= 50) {
                endY = kinematics.absoluteRobotTarget.position.x;
            } else {
                endY = 53;
            }
        } else {
            endY = 53;
        }

        double avgX = (current.getY() + endX) * 0.6;
        double avgY = (current.getX() + endY) * 0.6;

        Path s1 = simpleTangentLine(current, point(avgX, avgY)).getPath(0);
        Path s2 = simpleLine(point(avgX, avgY), point(endX, endY), -90).getPath(0);
        Path s3 = simpleLine(point(endX, endY), point(-13, endY), -90).getPath(0);

        PathBuilder builder = new PathBuilder();
        //builder.addPath(s1);
        //builder.addPath(s2);

        Path test = new Path(
                new BezierLine(
                        current, point(endX, endY)
                )
        );
        test.setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(-90));

        builder.addPath(test);
        builder.addPath(s3);
        return builder.build();
    }

    public static PathChain subToCV() {
        return simpleLine(submersible, cvStart, -90);
    }

    public static PathChain subToBasket() {
        return simpleReverseCurve(submersible, basketToSubControl, subToBasketControl, cvDump);
    }

    public static PathChain subToBasketFurther() {
        return simpleReverseCurve(submersible, basketToSubControl, subToBasketControl, cvDumpFurther);
    }

    public static PathChain cachedSubToCv(Limelight.SampleState cached) {
        if (cached.angle != 0 && cached.angle != 90) {
            Kinematics kinematics = new Kinematics(cached);
            if (kinematics.absoluteRobotTarget.position.x >= 50) {
                return simpleLine(
                        point(-8, kinematics.absoluteRobotTarget.position.x),
                        point(-13, kinematics.absoluteRobotTarget.position.x),
                        -90
                );
            } else {
                return simpleLine(point(-8, 53), point(-13, 53), -90);
            }
        } else {
            return simpleLine(point(-8, 53), point(-13, 53), -90);
        }
    }

    public static PathChain cachedStraightSubToCv(Point current, Limelight.SampleState cached) {
        if (cached.angle != 0 && cached.angle != 90) {
            Kinematics kinematics = new Kinematics(cached);
            if (kinematics.absoluteRobotTarget.position.x >= 50) {
                return simpleLine(
                        current,
                        point(-13, kinematics.absoluteRobotTarget.position.x),
                        -90
                );
            } else {
                return simpleLine(current, point(-13, 53), -90);
            }
        } else {
            return simpleLine(current, point(-13, 53), -90);
        }
    }
}
