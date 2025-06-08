package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import java.util.function.BooleanSupplier;

import core.subsystems.Intake;
import core.hardware.IndicatorLight;

public class PreloadWarning extends CommandBase {
    private Intake intakeSubsystem;
    private IndicatorLight light;
    private Timer timer;
    private BooleanSupplier endCondition;

    public PreloadWarning(Intake intakeSubsystem, IndicatorLight light, BooleanSupplier endCondition) {
        this.intakeSubsystem = intakeSubsystem;
        this.light = light;
        this.timer = new Timer();
        this.endCondition = endCondition;
    }

    @Override
    public void execute() {
        if (this.timer.getElapsedTimeSeconds() > 1) this.timer.resetTimer();
        if (!this.intakeSubsystem.isSlideLatched()) {
            if (this.timer.getElapsedTimeSeconds() < 0.5) this.light.setColour(0.28);
            if (this.timer.getElapsedTimeSeconds() > 0.5) this.light.setColour(1);
        } else {
            this.light.setColour(0.5);
        }
    }

    @Override
    public boolean isFinished() {
        return this.endCondition.getAsBoolean();
    }
}
