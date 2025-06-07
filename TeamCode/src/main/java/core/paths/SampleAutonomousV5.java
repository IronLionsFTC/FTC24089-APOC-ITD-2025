package core.paths;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import core.computerVision.Limelight;

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

    public static Point stageOne = point(17.5, 6);
    public static Point lastDump = point(18.5, 5.5);

    public static Point cvDump = point(17, 8.5);
    public static Point stageTwo = point(22.5, 8.5);
    public static Point dumpTwo = point(21.5, 7.5);
    public static Point stageThree = point(20.69, 7.5);

    public static Point submersible = point(-10, 53);
    public static Point basketToSubControl = point(15, 50);
    public static Point subToBasketControl = point(13, 15);
    public static Point cvStart = point(-12, 53);

    public static PathChain firstDumpAndPickup() {
        return simpleLine(start, stageOne, -20);
    }

    public static PathChain secondDumpAndPickup() {
        return simpleLine(stageOne, stageTwo, -9.5);
    }

    public static PathChain SecondPreplacedDump() {
        return simpleLine(stageTwo, dumpTwo, -13);
    }

    public static PathChain thirdDumpAndPickup() {
        return simpleLine(stageTwo, stageThree, 17.37);
    }

    public static PathChain lastDump() {
        return simpleLine(stageThree, lastDump, -20);
    }

    public static PathChain basketToSub() {
        return simpleCurve(stageOne, basketToSubControl, submersible);
    }

    public static PathChain cachedBasketToSub(Limelight.SampleState cached) {
        if (cached.angle != 0 && cached.angle != 90) {
            return simpleCurve(stageOne, basketToSubControl,
                    point(cached.robotPosition.y, cached.robotPosition.x)
            );
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
}
