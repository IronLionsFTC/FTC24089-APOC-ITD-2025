package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import core.state.Subsystems;
import core.subsystems.Outtake;

public class ClipSpecimen extends CommandBase {
    private Outtake outtakeSubsystem;
    private Timer timer;

    private double time;

    public ClipSpecimen(Outtake outtakeSubsystem, double time) {
        this.outtakeSubsystem = outtakeSubsystem;
        timer = new Timer();
        this.time = time;
    }

    @Override
    public void initialize() {
        this.outtakeSubsystem.state = Subsystems.OuttakeState.SpecimenOuttakeExit;
        timer.resetTimer();
    }

    @Override
    public void execute() {
        if (timer.getElapsedTimeSeconds() > time) this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawOpen;
    }

    @Override
    public boolean isFinished() {
        return this.timer.getElapsedTimeSeconds() > time + 0.2;
    }
}
