package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import core.state.Subsystems;
import core.subsystems.Intake;

public class WaitAndGrabSample extends CommandBase {
    private Intake intakeSubsystem;
    private Timer timer;

    public WaitAndGrabSample(Intake intakeSubsystem) {
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
        } else {
            this.intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawDown;
        }

        if (this.timer.getElapsedTimeSeconds() > 100.5) this.intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawGrabbing;
    }

    @Override
    public boolean isFinished() {
        return this.intakeSubsystem.hasClawClosed();
    }
}
