package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import core.state.Subsystems;
import core.subsystems.Outtake;

public class ClipSpecimen extends CommandBase {
    private Outtake outtakeSubsystem;
    private Timer timer;

    public ClipSpecimen(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        this.outtakeSubsystem.state = Subsystems.OuttakeState.SpecimenOuttakeExit;
        timer.resetTimer();
    }

    @Override
    public void execute() {
        if (timer.getElapsedTimeSeconds() > 0.5) this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawOpen;
    }

    @Override
    public boolean isFinished() {
        return this.timer.getElapsedTimeSeconds() > 0.6;
    }
}
