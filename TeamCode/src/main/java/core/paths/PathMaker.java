package core.paths;

import static core.paths.SampleAutonomousV5.currentToCV;
import static core.paths.SampleAutonomousV5.point;
import static core.paths.SampleAutonomousV5.simpleLine;
import static core.paths.SampleAutonomousV5.testCVDump;
import static core.paths.SampleAutonomousV5.testCVDumpStart;

import com.pedropathing.follower.Follower;
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
        Point basket = point(18 + this.offset * 0.3, 9 + this.offset * 0.3);
        Path CVToBasket = simpleLine(testCVDumpStart, basket, -30).getPath(0);

        PathBuilder builder = new PathBuilder();
        currentToCV.setConstantHeadingInterpolation(Math.toRadians(-30));
        CVToBasket.setConstantHeadingInterpolation(Math.toRadians(-30));
        builder.addPath(currentToCV);
        builder.addPath(CVToBasket);

        this.result = builder.build();
    }
}
