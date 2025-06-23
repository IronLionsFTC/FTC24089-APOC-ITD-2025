package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import core.subsystems.Intake;

public class RetractIntakeSlightly extends CommandBase {
    private Intake intakeSubsystem;
    private Timer timer;
    private boolean done;

    public RetractIntakeSlightly(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.timer = new Timer();
        this.done = false;
    }

    @Override
    public void initialize() {
        if (this.intakeSubsystem.getExtension() > 300) {
            this.intakeSubsystem.setExtension(this.intakeSubsystem.getExtension() - 100);
        } else {
            this.intakeSubsystem.setExtension(this.intakeSubsystem.getExtension() + 100);
        }
        this.timer.resetTimer();
    }

    @Override
    public void execute() {
        if (this.timer.getElapsedTimeSeconds() > 0.2) {
            this.done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return this.done;
    }
}
