package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.subsystems.Outtake;

public class TeleOpHangButton extends CommandBase {
    private Outtake outtake;

    public TeleOpHangButton(Outtake outtake) {
        this.outtake = outtake;
    }

    @Override
    public void initialize() {
        this.outtake.hasHung = true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
