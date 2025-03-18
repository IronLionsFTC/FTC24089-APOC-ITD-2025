package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class TeleopAutomaticTransfer extends CommandBase {
    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;

    public TeleopAutomaticTransfer(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public void execute() {
        if (this.intakeSubsystem.isSlideLatched() && this.intakeSubsystem.state == Subsystems.IntakeState.RetractedClawClosed) {
            if (this.outtakeSubsystem.state == Subsystems.OuttakeState.DownClawOpen) {
                this.outtakeSubsystem.nextState();
            }
        }
        if (this.intakeSubsystem.state == Subsystems.IntakeState.RetractedClawClosed
                && this.outtakeSubsystem.state == Subsystems.OuttakeState.DownClawClosed
                && this.outtakeSubsystem.clawClosed()) {
            this.intakeSubsystem.nextState();
            this.outtakeSubsystem.transferComplete = true;
        }

        if (this.outtakeSubsystem.state == Subsystems.OuttakeState.UpClawClosed && this.intakeSubsystem.state == Subsystems.IntakeState.RetractedClawClosed) {
            this.intakeSubsystem.nextState();
        }

        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing && outtakeSubsystem.state == Subsystems.OuttakeState.DownClawClosed) {
            outtakeSubsystem.state = Subsystems.OuttakeState.DownClawOpen;
        }
    }
}
