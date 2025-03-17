package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import core.computerVision.Limelight;

public class ScanForSample extends CommandBase {
    private Follower follower;
    private Limelight limelight;
    private Limelight.SampleState result;

    public ScanForSample(Follower follower, Limelight limelight, Limelight.SampleState buffer) {
        this.follower = follower;
        this.limelight = limelight;
        this.result = buffer;
    }

    @Override
    public void initialize() {
        this.limelight.enable();
    }

    @Override
    public void execute() {
        this.result = limelight.query();
    }

    @Override
    public boolean isFinished() {
        return this.result != null;
    }

    @Override
    public void end(boolean i) {
        this.limelight.disable();
        if (!i) this.follower.breakFollowing();
        if (!i) this.follower.holdPoint(follower.getPose());
    }
}
