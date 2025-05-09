package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.computerVision.Limelight;

public class RaiseLimelight extends CommandBase {
    private Limelight limelight;

    public RaiseLimelight(Limelight limelight) {
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        this.limelight.raise();
    }

    @Override
    public boolean isFinished() {
        return this.limelight.isRaised();
    }
}
