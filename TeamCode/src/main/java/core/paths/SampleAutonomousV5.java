package core.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import core.computerVision.Limelight;
import core.math.Kinematics;

public class SampleAutonomousV5 {
    private static Point point(double x, double y) {
        return new Point(y, x, Point.CARTESIAN);
    }

    private static PathChain simpleLine(Point a, Point b, double h) {
        Path path = new Path(
                new BezierLine(a, b)
        );
        path.setConstantHeadingInterpolation(Math.toRadians(h));
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

    public static Point stageOne = point(17.5, 7);
    public static Point controlOne = point(18.5, -3);
    public static Point lastDump = point(18.5, 5.5);
    public static Point cvDump = point(14, 13.5);
    public static Point cvDumpFurther = point(13, 11.5);
    public static Point stageTwo = point(22.5, 9);
    public static Point dumpTwo = point(21.5, 7.5);
    public static Point stageThree = point(20.69, 8.5);
    public static Point submersible = point(-10, 53);
    public static Point basketToSubControl = point(15, 50);
    public static Point subToBasketControl = point(11, 17);
    public static Point cvStart = point(-13, 53);

    public static PathChain firstDumpAndPickup() {
        return constantCurve(start, controlOne, stageOne, -19);
    }

    public static PathChain secondDumpAndPickup() {
        return simpleLine(stageOne, stageTwo, -11.8);
    }

    public static PathChain SecondPreplacedDump() {
        return simpleLine(stageTwo, dumpTwo, -13);
    }

    public static PathChain thirdDumpAndPickup() {
        return simpleLine(stageTwo, stageThree, 17.5);
    }

    public static PathChain lastDump() {
        return simpleLine(stageThree, lastDump, -20);
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
                        point(-10, kinematics.absoluteRobotTarget.position.x),
                        point(-13, kinematics.absoluteRobotTarget.position.x),
                        -90
                );
            } else {
                return subToCV();
            }
        } else {
            return subToCV();
        }
    }
}
