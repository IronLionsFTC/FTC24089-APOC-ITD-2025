package core.paths;

import static core.paths.SampleAutonomousV5.currentToCV;
import static core.paths.SampleAutonomousV5.point;
import static core.paths.SampleAutonomousV5.simpleTangentLine;
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
        Point basket = point(18 + this.offset * 0.3, 11 + this.offset * 0.3);
        Path CVToBasket = simpleTangentLine(testCVDumpStart, basket).getPath(0);

        PathBuilder builder = new PathBuilder();
        builder.addPath(currentToCV);
        builder.addPath(CVToBasket);
        builder.setReversed(true);

        this.result = builder.build();
    }
}
