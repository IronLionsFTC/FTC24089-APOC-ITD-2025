package core.paths;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class SpecimenAutonomous {
    private static Point point(double x, double y) {
        return new Point(x, y, Point.CARTESIAN);
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
    public static Point firstDump = point(30.4, 7);

    public static Point spikeOne = point(26.5, -14.5);
    public static Point spikeTwo = point(26.5, -25);
    public static Point spikeThree = point(32, -33.3);

    public static Point hpOne = point(17.2, -14.3);
    public static Point hpTwo = point(16.4, -18.2);
    public static Point hpThree = point(16.6, -26.2);

    public static Point pickupA = point(5, -32);
    public static Point pickupB = point(5, -32);
    public static Point pickupC = point(5, -32);
    public static Point pickupD = point(5, -32);

    public static Point park = point(6, -25);
    public static Point intermediary = point(20, 5);

    public static Point dumpA = point(30, 8.36);
    public static Point dumpB = point(30, 8.36);
    public static Point dumpC = point(30, 8.36);
    public static Point dumpD = point(30, 8.36);

    public static PathChain park() {
        return simpleLine(dumpC, park, 0);
    }

    public static PathChain firstDump() {
        return simpleLine(start, firstDump, 0);
    }
    public static PathChain firstSpike() {
        PathBuilder builder = new PathBuilder();

        Path turnAway = new Path(
                new BezierLine(
                        firstDump,
                        intermediary
                )
        );

        turnAway.setLinearHeadingInterpolation(0, -1.2);

        Path goToSample = new Path(
                new BezierLine(
                        intermediary,
                        spikeOne
                )
        );

        goToSample.setConstantHeadingInterpolation(-1.15);

        builder.addPath(
            turnAway
        );

        builder.addPath(
                goToSample
        );

        return builder.build();
    }

    public static PathChain firstHp() {
        return simpleLine(spikeOne, hpOne, -131.78);
    }

    public static PathChain secondSpike() {
        return simpleLine(hpOne, spikeTwo, Math.toDegrees(-1.1));
    }

    public static PathChain secondHp() {
        return simpleLine(spikeTwo, hpTwo, Math.toDegrees(-2.3));
    }

    public static PathChain thirdSpike() {
        return simpleLine(hpTwo, spikeThree, Math.toDegrees(-1.3));
    }

    public static PathChain thirdHp() {
        return simpleLine(spikeThree, hpThree, Math.toDegrees(-2.4));
    }

    public static PathChain startCycling() {
        return simpleLine(hpThree, pickupA, 0);
    }

    public static PathChain goDumpA() {
        return simpleLine(pickupA, dumpA, Math.toDegrees(0));
    }

    public static PathChain returnA() {
        return simpleLine(dumpA, pickupB, 0);
    }

    public static PathChain goDumpB() {
        return simpleLine(pickupB, dumpB, Math.toDegrees(0));
    }

    public static PathChain returnB() {
        return simpleLine(dumpB, pickupC, 0);
    }


    public static PathChain goDumpC() {
        return simpleLine(pickupC, dumpC, Math.toDegrees(0));
    }

    public static PathChain returnC() {
        return simpleLine(dumpC, pickupD, 0);
    }


    public static PathChain goDumpD() {
        return simpleLine(pickupD, dumpD, Math.toDegrees(0));
    }

    public static PathChain returnD() {
        return simpleLine(dumpD, park, 0);
    }
}
