package core.commands;

import static core.paths.SampleAutonomousV5.point;
import static core.paths.SampleAutonomousV5.simpleLine;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Timer;

public class BackForth extends CommandBase {
    private Follower follower;
    private int path;
    private PathChain pathChain;
    private Timer timer;
    private boolean waiting;

    public BackForth(Follower follower) {
        this.timer = new Timer();
        this.follower = follower;
        this.path = 0;
    }

    @Override
    public void initialize() {
        this.timer.resetTimer();
        PathBuilder builder = new PathBuilder();
        Path a = simpleLine(point(0, 0), point(0, 40), 0).getPath(0);
        Path b = simpleLine(point(0, 40), point(0, 0), 0).getPath(0);
        builder.addPath(a);
        builder.addPath(b);
        this.pathChain = builder.build();
        this.waiting = false;
    }

    @Override
    public void execute() {
        if (this.follower.getCurrentTValue() > 0.95 && !this.waiting) {
            this.path = 1 - this.path;
            this.waiting = true;
            this.timer.resetTimer();
        }

        if (this.waiting && this.timer.getElapsedTimeSeconds() > 1) {
            if (this.path == 0) {
                follower.followPath(pathChain.getPath(0), true);
            } else {
                follower.followPath(pathChain.getPath(1), true);
            }
            this.waiting = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
