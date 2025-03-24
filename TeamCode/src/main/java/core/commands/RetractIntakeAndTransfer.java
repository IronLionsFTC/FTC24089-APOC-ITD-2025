package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.parameters.HardwareParameters;
import core.parameters.PositionalBounds;
import core.state.Subsystems;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class RetractIntakeAndTransfer extends CommandBase {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;

    public RetractIntakeAndTransfer(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        // Ensure the intake state is grabbing, and if so bring it in for transfer
        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing) this.intakeSubsystem.nextState();
        // If outtake is not ready for transfer, abort
        this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawOpen;
        this.outtakeSubsystem.transferComplete = false;
    }

    @Override
    public void execute() {

        // If the intake slides have been in for a certain period of time
        if (this.intakeSubsystem.isSlideLatched()) {
            // Grab with the outtake claw
            if (this.outtakeSubsystem.state != Subsystems.OuttakeState.DownClawClosed) this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawClosed;
            // If the outtake claw has closed, release the intake claw
            if (this.outtakeSubsystem.clawClosed()) {
                this.intakeSubsystem.state = Subsystems.IntakeState.RetractedClawOpen;
                this.outtakeSubsystem.transferComplete = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.state == Subsystems.OuttakeState.DownClawClosed && this.intakeSubsystem.clawOpen();
    }
}
