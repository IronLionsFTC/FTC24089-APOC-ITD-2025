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
    private boolean done;

    public RetractIntakeAndTransfer(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.done = false;
    }

    @Override
    public void initialize() {
        // Ensure the intake state is grabbing, and if so bring it in for transfer
        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing) this.intakeSubsystem.nextState();
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
                if (this.intakeSubsystem.clawOpen()) {
                    this.outtakeSubsystem.transferComplete = true;
                    this.done = true;
                }
            }
        } else {
            this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawOpen;
            done = false;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
