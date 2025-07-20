package core.paths;

import static core.paths.SampleAutonomousV5.currentToCV;
import static core.paths.SampleAutonomousV5.point;
import static core.paths.SampleAutonomousV5.simpleTangentLine;
import static core.paths.SampleAutonomousV5.testCVDumpStart;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class PathMaker {
    public int offset;
    public PathChain result;

    public PathMaker() {
        this.offset = 0;
    }

    public void increment() {
        this.offset += 1;
    }

    public void calculate(Follower follower) {
        Path currentToCV = currentToCV(follower);
        Point basket = point(18 + this.offset * 0.3, 10 + this.offset * 0.3);
        Path CVToBasket = simpleTangentLine(testCVDumpStart, basket).getPath(0);

        PathBuilder builder = new PathBuilder();
        builder.addPath(currentToCV);
        builder.addPath(CVToBasket);
        builder.setReversed(true);

        this.result = builder.build();
    }

    public void ncalculate(Follower follower) {
        Pose current = follower.getPose();
        Point currentPoint = point(current.getY(), current.getX());
        Point basket = point(18 + this.offset * 0.3, 10 + this.offset * 0.3);
        // -4.5577 X | 10.68 Y
        // -18.691 X | 16.97 Y
        Point c1 = point(current.getY() + 10.68, current.getX() - 4.5577);
        Point c2 = point(current.getY() + 16.97, current.getX() - 18.691);
        PathBuilder builder = new PathBuilder();
        Path curve = new Path(new BezierCurve(currentPoint, c1, c2, basket));
        curve.setTangentHeadingInterpolation();
        builder.addPath(curve);
        builder.setTangentHeadingInterpolation();
        builder.setReversed(true);
        this.result = builder.build();
    }
}
