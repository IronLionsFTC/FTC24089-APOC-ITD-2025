package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.state.Subsystems;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class TeleopOverride extends CommandBase {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;

    public TeleopOverride(Intake intakeSubsystem, Outtake outtakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing) {
            // Open the claw if a grab is missed
            this.intakeSubsystem.cancelGrab();
        } else if (this.outtakeSubsystem.state == Subsystems.OuttakeState.UpClawClosed) {
            // If the outtake is up ready to deposit, then the override button switches buckets
            this.outtakeSubsystem.toggleBasket();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
