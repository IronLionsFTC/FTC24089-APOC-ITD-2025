package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Outtake;

public class SlamDunkSample extends CommandBase {
    private Outtake outtakeSubsystem;

    public SlamDunkSample(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void initialize() {
        if (outtakeSubsystem.state == Subsystems.OuttakeState.UpClawClosed) outtakeSubsystem.nextState();
    }

    @Override
    public boolean isFinished() {
        if (this.outtakeSubsystem.clawOpened()) {
            this.outtakeSubsystem.nextState();
            return true;
        } else {
            return false;
        }
    }
}
