package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Outtake;

public class ReachForWallSpecimen extends CommandBase {
    private Outtake outtakeSubsystem;

    public ReachForWallSpecimen(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        this.outtakeSubsystem.state = Subsystems.OuttakeState.SpecimenIntakeClawOpen;
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.armPhysicallyOver();
    }
}
