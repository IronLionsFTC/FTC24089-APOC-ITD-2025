package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import core.computerVision.Limelight;
import core.math.Kinematics;
import core.state.Subsystems;
import core.subsystems.Intake;

public class ExtendSlidesForSample extends CommandBase {
    private Intake intakeSubsystem;
    private Limelight.SampleState buffer;

    public ExtendSlidesForSample(Intake intakeSubsystem, Limelight.SampleState buffer) {
        this.intakeSubsystem = intakeSubsystem;
        this.buffer = buffer;
    }

    @Override
    public void initialize() {
        Kinematics kinematics = new Kinematics(buffer);
        intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawDown;
        intakeSubsystem.setExtension(kinematics.absoluteSlidePosition);
    }

    @Override
    public boolean isFinished() {
        return this.intakeSubsystem.isSlidesExtended();
    }
}
