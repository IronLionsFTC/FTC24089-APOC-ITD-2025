package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.computerVision.Limelight;
import core.parameters.PositionalBounds;
import core.subsystems.Intake;

public class AlignClaw extends CommandBase {
    private Intake intakeSubsystem;
    private Limelight.SampleState buffer;

    public AlignClaw(Intake intakeSubsystem, Limelight.SampleState buffer) {
        this.intakeSubsystem = intakeSubsystem;
        this.buffer = buffer;
    }

    @Override
    public void initialize() {
        if (this.buffer == null) return;
        this.intakeSubsystem.setIntakeClawRotation(PositionalBounds.ServoPositions.ClawPositions.yawRest - buffer.angle / 355);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean i) {
        if (!i) this.buffer.angle = 0;
    }
}
