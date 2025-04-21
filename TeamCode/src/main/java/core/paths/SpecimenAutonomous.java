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
    public static Point firstDump = point(31, 0);
    public static Point spikeOne = point(28.7, -12);
    public static Point spikeTwo = point(26.2, -23.7);
    public static Point spikeThree = point(32, -33.3);

    public static Point hpOne = point(17.2, -14.3);
    public static Point hpTwo = point(16.4, -18.2);
    public static Point hpThree = point(16.6, -26.2);

    public static Point pickup = point(16.6, -16);

    public static Point intermediary = point(20, 0);

    public static Point dumpA = point(31, 7.36);
    public static Point dumpB = point(32, 9.36);
    public static Point dumpC = point(32, 11.36);
    public static Point dumpD = point(33, 13.36);

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

        goToSample.setConstantHeadingInterpolation(-1.2);

        builder.addPath(
            turnAway
        );

        builder.addPath(
                goToSample
        );

        return builder.build();
    }

    public static PathChain firstHp() {
        return simpleLine(spikeOne, hpOne, Math.toDegrees(-2.3));
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
        return simpleLine(hpThree, pickup, Math.toDegrees(-2.4));
    }

    public static PathChain goDumpA() {
        return simpleLine(pickup, dumpA, Math.toDegrees(0));
    }

    public static PathChain returnA() {
        PathBuilder builder = new PathBuilder();

        Path turnAway = new Path(
                new BezierLine(
                        dumpA,
                        intermediary
                )
        );

        turnAway.setLinearHeadingInterpolation(0, -2);

        Path goToSample = new Path(
                new BezierLine(
                        intermediary,
                        pickup
                )
        );

        goToSample.setConstantHeadingInterpolation(-2.4);

        builder.addPath(
                turnAway
        );

        builder.addPath(
                goToSample
        );

        return builder.build();
    }

    public static PathChain goDumpB() {
        return simpleLine(pickup, dumpB, Math.toDegrees(0));
    }

    public static PathChain returnB() {
        PathBuilder builder = new PathBuilder();

        Path turnAway = new Path(
                new BezierLine(
                        dumpB,
                        intermediary
                )
        );

        turnAway.setLinearHeadingInterpolation(0, -2);

        Path goToSample = new Path(
                new BezierLine(
                        intermediary,
                        pickup
                )
        );

        goToSample.setConstantHeadingInterpolation(-2.4);

        builder.addPath(
                turnAway
        );

        builder.addPath(
                goToSample
        );

        return builder.build();
    }


    public static PathChain goDumpC() {
        return simpleLine(pickup, dumpC, Math.toDegrees(0));
    }

    public static PathChain returnC() {
        PathBuilder builder = new PathBuilder();

        Path turnAway = new Path(
                new BezierLine(
                        dumpC,
                        intermediary
                )
        );

        turnAway.setLinearHeadingInterpolation(0, -2);

        Path goToSample = new Path(
                new BezierLine(
                        intermediary,
                        pickup
                )
        );

        goToSample.setConstantHeadingInterpolation(-2.4);

        builder.addPath(
                turnAway
        );

        builder.addPath(
                goToSample
        );

        return builder.build();
    }


    public static PathChain goDumpD() {
        return simpleLine(pickup, dumpD, Math.toDegrees(0));
    }

    public static PathChain returnD() {
        PathBuilder builder = new PathBuilder();

        Path turnAway = new Path(
                new BezierLine(
                        dumpD,
                        intermediary
                )
        );

        turnAway.setLinearHeadingInterpolation(0, -2);

        Path goToSample = new Path(
                new BezierLine(
                        intermediary,
                        pickup
                )
        );

        goToSample.setConstantHeadingInterpolation(-2.4);

        builder.addPath(
                turnAway
        );

        builder.addPath(
                goToSample
        );

        return builder.build();
    }
}
