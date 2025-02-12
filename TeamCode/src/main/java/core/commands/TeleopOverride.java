package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.state.Subsystems;
import core.subsystems.Intake;

public class TeleopOverride extends CommandBase {

    private final Intake intakeSubsystem;

    public TeleopOverride(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing) {
            this.intakeSubsystem.cancelGrab();
        }
    }
}
