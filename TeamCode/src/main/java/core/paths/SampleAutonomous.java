package core.paths;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class SampleAutonomous {

    private static Point point(double x, double y) {
        return new Point(-x / 25.4, y / 25.4, Point.CARTESIAN);
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
    public static final Point dump = point(850, 200);
    public static final Point preloadCurve = point(750, 300);

    public static PathChain dumpPreload() {
        Path path = new Path(
                new BezierCurve(
                        start,
                        preloadCurve,
                        dump
                )
        );

        path.setTangentHeadingInterpolation();
        path.setReversed(true);

        PathBuilder builder = new PathBuilder();
        builder.addPath(path);

        return builder.build();
    }
}
