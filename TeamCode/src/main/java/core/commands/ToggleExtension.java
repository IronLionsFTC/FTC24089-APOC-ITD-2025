package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.parameters.PositionalBounds;
import core.subsystems.Intake;

public class ToggleExtension extends CommandBase {
    private Intake intakeSubsystem;

    public ToggleExtension(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        if (this.intakeSubsystem.getExtension() < 500) this.intakeSubsystem.setExtension(
                PositionalBounds.SlidePositions.IntakePositions.extended
        );
        else this.intakeSubsystem.setExtension(450);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
