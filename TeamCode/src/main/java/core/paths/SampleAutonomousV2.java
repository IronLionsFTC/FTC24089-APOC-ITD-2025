package core.paths;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class SampleAutonomousV2 {
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
    public static Point stageOne = point(19, 13);
    public static Point stageTwo = point(22.5, 13);
    public static Point stageThree = point(17, 12.5);

    public static Point submersible = point(0, 55);
    public static Point basketToSubControl = point(15, 50);
    public static Point subToBasketControl = point(10, 15);
    public static Point cvStart = point(-5, 53);

    public static PathChain firstDumpAndPickup() {
        return simpleLine(start, stageOne, -22.9);
    }

    public static PathChain secondDumpAndPickup() {
        return simpleLine(stageOne, stageTwo, -8);
    }

    public static PathChain thirdDumpAndPickup() {
        return simpleLine(stageTwo, stageThree, 27);
    }

    public static PathChain lastDump() {
        return simpleLine(stageThree, stageOne, -22.9);
    }

    public static PathChain basketToSub() {
        return simpleCurve(stageOne, basketToSubControl, submersible);
    }

    public static PathChain subToCV() {
        return simpleLine(submersible, cvStart, -90);
    }

    public static PathChain subToBasket() {
        return simpleReverseCurve(submersible, basketToSubControl, subToBasketControl, stageOne);
    }
}
