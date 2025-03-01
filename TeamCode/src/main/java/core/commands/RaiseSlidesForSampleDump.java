package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Outtake;

public class RaiseSlidesForSampleDump extends CommandBase {
    private Outtake outtakeSubsystem;

    public RaiseSlidesForSampleDump(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void initialize() {
        if (this.outtakeSubsystem.state == Subsystems.OuttakeState.DownClawClosed) outtakeSubsystem.nextState();
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.atTargetHeight();
    }
}
