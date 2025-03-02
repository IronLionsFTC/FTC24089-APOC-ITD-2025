package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Outtake;

public class GrabWallSpecimenAndFlip extends CommandBase {
    private Outtake outtakeSubsystem;

    public GrabWallSpecimenAndFlip(Outtake outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        this.outtakeSubsystem.state = Subsystems.OuttakeState.SpecimenIntakeClawClosed;
    }

    @Override
    public void execute() {
        if (this.outtakeSubsystem.clawClosed() && this.outtakeSubsystem.state == Subsystems.OuttakeState.SpecimenIntakeClawClosed) {
            this.outtakeSubsystem.nextState();
        }
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.armPhysicallyDown() && this.outtakeSubsystem.state == Subsystems.OuttakeState.SpecimenOuttakeEntry;
    }
}
