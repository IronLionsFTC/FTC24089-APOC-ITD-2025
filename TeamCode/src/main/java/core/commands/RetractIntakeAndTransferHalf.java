package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import core.state.Subsystems;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class RetractIntakeAndTransferHalf extends CommandBase {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Timer deadMan;
    private Timer total;
    private boolean up;

    public RetractIntakeAndTransferHalf(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.deadMan = new Timer();
        this.total = new Timer();
        this.up = false;
    }

    @Override
    public void initialize() {
        // Ensure the intake state is grabbing, and if so bring it in for transfer
        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing) this.intakeSubsystem.nextState();
        this.intakeSubsystem.half();
        // If outtake is not ready for transfer, abort
        this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawOpen;
        this.outtakeSubsystem.transferComplete = false;
        this.deadMan.resetTimer();
        this.total.resetTimer();
        this.outtakeSubsystem.pitchUp = true;
    }

    @Override
    public void execute() {
        if (!this.intakeSubsystem.isSlidesRetracted()) this.deadMan.resetTimer();

        if (this.intakeSubsystem.isSlidesRetracted() && !this.up) {
            this.intakeSubsystem.full();
            this.up = true;
        }

        if (!this.up) this.deadMan.resetTimer();

        if (this.deadMan.getElapsedTimeSeconds() > 0.2) {
            this.outtakeSubsystem.pitchUp = false;
        }

        if (this.deadMan.getElapsedTimeSeconds() > 0.4) {
            this.outtakeSubsystem.state = Subsystems.OuttakeState.DownClawClosed;
        }

        if (this.outtakeSubsystem.clawClosed() && intakeSubsystem.state == Subsystems.IntakeState.RetractedClawClosed) {
            this.outtakeSubsystem.transferComplete = true;
            this.intakeSubsystem.nextState();
        }
    }

    @Override
    public boolean isFinished() {
        return this.outtakeSubsystem.transferComplete || this.total.getElapsedTimeSeconds() > 4.5;
    }

    @Override
    public void end(boolean i) {
        outtakeSubsystem.pitchUp = false;
    }
}
