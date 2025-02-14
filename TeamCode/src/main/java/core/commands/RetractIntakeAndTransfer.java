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
    private boolean transferComplete;
    private int positionalStability;

    public RetractIntakeAndTransfer(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.transferComplete = false;
        this.positionalStability = 0;
    }

    @Override
    public void initialize() {
        // Ensure the intake state is grabbing, and if so bring it in for transfer
        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing) this.intakeSubsystem.nextState();
        // If outtake is not ready for transfer, make sure it is
        if (this.outtakeSubsystem.state != Subsystems.OuttakeState.DownClawOpen) this.intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawGrabbing;
    }

    @Override
    public void execute() {
        if (this.intakeSubsystem.slideExtension() < 0.05) positionalStability += 1;
        else positionalStability = 0;

        // If the intake slides have been in for a certain period of time
        if (positionalStability > 20) {
            // Grab with the outtake claw
            if (this.outtakeSubsystem.state == Subsystems.OuttakeState.DownClawOpen) this.outtakeSubsystem.nextState();
            // If the outtake claw has closed, release the intake claw
            if (this.outtakeSubsystem.state == Subsystems.OuttakeState.DownClawClosed && this.outtakeSubsystem.clawOpened()) this.intakeSubsystem.nextState();
        }
    }
}
