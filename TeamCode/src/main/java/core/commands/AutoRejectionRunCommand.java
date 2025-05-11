package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.state.Subsystems;
import core.subsystems.Intake;

public class AutoRejectionRunCommand extends CommandBase {
    private Intake intakeSubsystem;
    private Telemetry telemetry;

    public AutoRejectionRunCommand(Intake intakeSubsystem, Telemetry telemetry) {
        this.intakeSubsystem = intakeSubsystem;
        this.telemetry = telemetry;
    }

    @Override
    public void execute() {
        telemetry.addData("Intake Has Sample", this.intakeSubsystem.hasIntakeGotSample());
        if (this.intakeSubsystem.state == Subsystems.IntakeState.ExtendedClawGrabbing) {
            if (!this.intakeSubsystem.isClawHoveringOverSample()) {
                this.intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawDown;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
