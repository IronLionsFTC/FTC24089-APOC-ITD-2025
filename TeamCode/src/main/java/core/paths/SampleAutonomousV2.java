package core.paths;

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

    public static Point start = point(0, 0);
    public static Point stageOne = point(18, 14);
    public static Point stageTwo = point(23, 13);
    public static Point stageThree = point(23, 13);

    public static PathChain firstDumpAndPickup() {
        return simpleLine(start, stageOne, -23);
    }

    public static PathChain secondDumpAndPickup() {
        return simpleLine(stageOne, stageTwo, -10);
    }

    public static PathChain thirdDumpAndPickup() {
        return simpleLine(stageTwo, stageThree, 10);
    }
}
