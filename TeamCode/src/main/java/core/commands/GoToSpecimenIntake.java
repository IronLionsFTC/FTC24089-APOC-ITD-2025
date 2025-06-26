package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import core.state.Subsystems;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class GoToSpecimenIntake extends CommandBase {
    private Outtake outtakeSubsystem;
    private Timer timer;
    private boolean done;

    public GoToSpecimenIntake(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.timer = new Timer();
        this.done = false;
    }

    @Override
    public void initialize() {
        this.outtakeSubsystem.state = Subsystems.OuttakeState.SpecimenIntakeClawOpen;
    }

    @Override
    public void execute() {
        if (outtakeSubsystem.state == Subsystems.OuttakeState.SpecimenIntakeClawOpen && this.outtakeSubsystem.armPhysicallyDown()) {
            this.done = true;
        }

        if (!this.done) timer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return this.done && this.timer.getElapsedTimeSeconds() > 0.5 && this.outtakeSubsystem.clawOpened();
    }
}
