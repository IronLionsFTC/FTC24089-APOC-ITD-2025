package core.paths;

import static core.paths.SampleAutonomousV5.point;
import static core.paths.SampleAutonomousV5.simpleLine;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class EightSampleAuto {
    public static Point start = point(0, 0);
    public static Point grab1 = point(17, 7.6);
    public static Point grab2 = point(22.5, 9);
    public static Point grab3 = point(20.69, 8.5);
    public static Point dump4 = point(17.5, 7);

    public static PathChain stage1() {
        PathBuilder builder = new PathBuilder();

        Path path = new Path(
                new BezierLine(
                        start,
                        grab1
                )
        );

        path.setConstantHeadingInterpolation(Math.toRadians(-20));
        builder.addPath(path);

        return builder.build();
    }

    public static PathChain stage2() { return simpleLine(grab1, grab2,  -12); }
    public static PathChain stage3() { return simpleLine(grab2, grab3,   17); }
    public static PathChain stage4() { return simpleLine(grab3, dump4,  -30); }
}
