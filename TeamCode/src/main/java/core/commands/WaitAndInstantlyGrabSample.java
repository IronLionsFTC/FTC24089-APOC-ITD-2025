package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import core.state.Subsystems;
import core.subsystems.Intake;

public class WaitAndInstantlyGrabSample extends CommandBase {
    private Intake intakeSubsystem;
    private Timer timer;

    public WaitAndInstantlyGrabSample(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        this.timer.resetTimer();
    }

    @Override
    public void execute() {
        if (this.intakeSubsystem.isClawHoveringOverSample()) {
            this.intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawGrabbing;
        }
        if (this.timer.getElapsedTimeSeconds() > 1.5) this.intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawGrabbing;
    }

    @Override
    public boolean isFinished() {
        return this.intakeSubsystem.hasClawClosed();
    }
}
