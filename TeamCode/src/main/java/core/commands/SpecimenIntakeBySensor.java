package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import core.state.Subsystems;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class SpecimenIntakeBySensor extends CommandBase {
    private Outtake outtakeSubsystem;
    private Intake intakeSubsystem;
    private Timer timer;
    private boolean done;

    public SpecimenIntakeBySensor(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.timer = new Timer();
        this.done = false;
    }

    @Override
    public void execute() {
        if (this.intakeSubsystem.outtakeClawReady() && this.outtakeSubsystem.state == Subsystems.OuttakeState.SpecimenIntakeClawOpen) {
            this.outtakeSubsystem.nextState();
            this.done = true;
        }

        if (!this.done) timer.resetTimer();

        if (!this.intakeSubsystem.outtakeClawReady()) {
            this.done = false;
            this.outtakeSubsystem.state = Subsystems.OuttakeState.SpecimenIntakeClawOpen;
        }
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.clawClosed() && done && this.timer.getElapsedTimeSeconds() > 0.3;
    }
}
