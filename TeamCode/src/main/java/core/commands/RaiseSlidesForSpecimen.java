package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Outtake;

public class RaiseSlidesForSpecimen extends CommandBase {
    private Outtake outtakeSubsystem;

    public RaiseSlidesForSpecimen(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        this.outtakeSubsystem.state = Subsystems.OuttakeState.SpecimenOuttakeEntry;
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.armPhysicallyDown() && this.outtakeSubsystem.atTargetHeight();
    }
}
