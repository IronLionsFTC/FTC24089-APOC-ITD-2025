package core.paths;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class SampleAutonomous {

    private static Point point(double x, double y) {
        return new Point(-x / 25.4, y / 25.4, Point.CARTESIAN);
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

    /*
        Robot
       []      [] <- Basket

       |     - - - <- samples
       |
       |
       ^ Sub

       x ->

       y
       |
       v
     */

    public static final Point start = point(0, 0);
    public static final Point dump = point(360, 50);
    public static final Point grab1 = point(515, 250);
    public static final Point dump1 = point(600, 340);
    public static final Point grab2 = point(600, 250);
    public static final Point dump2 = point(750, 380);
    public static final Point grab3 = point(550, 360);
    public static final Point control = point(750, 1500);
    public static final Point control2 = point(600, 500);
    public static final Point park = point(-50, 1500);

    public static PathChain dumpPreload() {
        return simpleLine(start, dump, 10);
    }

    public static PathChain firstPreplaced() {
        Path path = new Path(
                new BezierLine(
                        dump,
                        grab1
                )
        );

        path.setConstantHeadingInterpolation(Math.toRadians(76));

        PathBuilder builder = new PathBuilder();
        builder.addPath(path);

        return builder.build();
    }

    public static PathChain firstDump() {
        Path path = new Path(
                new BezierLine(
                        grab1,
                        dump1
                )
        );

        path.setConstantHeadingInterpolation(Math.toRadians(60));

        PathBuilder builder = new PathBuilder();
        builder.addPath(path);

        return builder.build();
    }

    public static PathChain secondPreplaced() {
        Path path = new Path(
                new BezierLine(
                        dump1,
                        grab2
                )
        );

        path.setConstantHeadingInterpolation(Math.toRadians(92));
        PathBuilder builder = new PathBuilder();
        builder.addPath(path);
        return builder.build();
    }

    public static PathChain secondDump() {
        return simpleLine(grab2, dump2, 80);
    }

    public static PathChain thirdGrab() {
        return simpleLine(dump2, grab3, 120);
    }

    public static PathChain thirdDump() {
        return simpleLine(grab3, dump1, 60);
    }

    public static PathChain goToSub() {
        Path path = new Path(
                new BezierCurve(
                        dump1,
                        control,
                        park
                )
        );

        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }

    public static PathChain goToBasket() {
        Path path = new Path(
                new BezierCurve(
                        park,
                        control,
                        control2,
                        dump1
                )
        );

        path.setReversed(true);

        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(path);
        return pathBuilder.build();
    }
}
