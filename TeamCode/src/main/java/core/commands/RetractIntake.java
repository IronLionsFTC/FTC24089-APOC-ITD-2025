package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Intake;

public class RetractIntake extends CommandBase {
    private Intake intakeSubsystem;

    public RetractIntake(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        this.intakeSubsystem.state = Subsystems.IntakeState.RetractedClawOpen;
    }

    @Override
    public boolean isFinished() {
        return this.intakeSubsystem.isSlidesRetracted();
    }
}
