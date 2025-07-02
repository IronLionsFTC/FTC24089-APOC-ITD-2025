package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import core.state.Subsystems;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class RetractIntakeAndTransfer extends CommandBase {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Timer deadMan;
    private Timer total;

    public RetractIntakeAndTransfer(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.deadMan = new Timer();
        this.total = new Timer();
    }

    @Override
    public void initialize() {
        // Ensure the intake state is grabbing, and if so bring it in for transfer
        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing) this.intakeSubsystem.nextState();
        // If outtake is not ready for transfer, abort
        // this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawOpen;
        this.outtakeSubsystem.transferComplete = false;
        this.deadMan.resetTimer();
        this.total.resetTimer();
    }

    @Override
    public void execute() {

        // If the intake slides have been in for a certain period of time
        if (this.intakeSubsystem.isSlideLatched() && this.outtakeSubsystem.areSlidesDown()) {
            // Grab with the outtake claw
            if (this.outtakeSubsystem.state != Subsystems.OuttakeState.DownClawClosed) this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawClosed;
            // If the outtake claw has closed, release the intake claw
            if (this.outtakeSubsystem.clawClosed()) {
                this.intakeSubsystem.state = Subsystems.IntakeState.RetractedClawOpen;
                //if (this.intakeSubsystem.clawOpen()) outtakeSubsystem.transferComplete = true;
                outtakeSubsystem.transferComplete = true;
            }
        }
        if (!this.intakeSubsystem.isSlideLatched()) this.deadMan.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.transferComplete || this.total.getElapsedTimeSeconds() > 1.5;
    }
}
