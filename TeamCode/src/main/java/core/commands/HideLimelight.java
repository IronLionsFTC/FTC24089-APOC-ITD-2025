package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.computerVision.Limelight;

public class HideLimelight extends CommandBase {
    private Limelight limelight;

    public HideLimelight(Limelight limelight) {
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        this.limelight.hide();
    }

    @Override
    public boolean isFinished() {
        return this.limelight.isHidden();
    }
}
