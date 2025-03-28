package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Outtake;

public class RaiseSlidesForSampleDump extends CommandBase {
    private Outtake outtakeSubsystem;

    public RaiseSlidesForSampleDump(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        this.outtakeSubsystem.state = Subsystems.OuttakeState.UpClawClosed;
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.areSlidesRaised();
    }
}
