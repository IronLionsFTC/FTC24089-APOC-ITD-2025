package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.subsystems.Outtake;

public class WaitUntilOuttakeDown extends CommandBase {
    private Outtake outtakeSubsystem;

    public WaitUntilOuttakeDown(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.areSlidesDown();
    }
}
