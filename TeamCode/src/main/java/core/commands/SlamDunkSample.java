package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Outtake;

public class SlamDunkSample extends CommandBase {
    private Outtake outtakeSubsystem;

    public SlamDunkSample(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        this.outtakeSubsystem.state = Subsystems.OuttakeState.UpClawOpen;
    }

    @Override
    public boolean isFinished() {
        if (this.outtakeSubsystem.clawOpened()) {
            this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawOpen;
            return true;
        } else {
            return false;
        }
    }
}
