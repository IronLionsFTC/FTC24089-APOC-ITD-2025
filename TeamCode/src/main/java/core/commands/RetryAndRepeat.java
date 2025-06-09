package core.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Light;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.subsystems.Intake;
import core.subsystems.Outtake;
import core.computerVision.Limelight;

public class RetryAndRepeat extends CommandBase {
    final private Intake intakeSubsystem;
    final private Outtake outtakeSubsystem;
    final private Follower follower;
    final private Telemetry telemetry;
    final private Limelight limelight;
    final private Limelight.SampleState buffer;

    private boolean done;

    private Command fullSequence;

    public RetryAndRepeat(
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Follower follower,
            Telemetry telemetry,
            Limelight limelight,
            Limelight.SampleState buffer
    ) {
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.follower = follower;
        this.telemetry = telemetry;
        this.limelight = limelight;
        this.buffer = buffer;

        this.done = false;

        this.fullSequence = CMD.retractIntake(intakeSubsystem).alongWith(CMD.raiseLimelight(limelight)).andThen(
                CMD.sleep(500).andThen(
                    CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false).andThen(
                            CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer, telemetry).alongWith(
                                    CMD.alignClaw(intakeSubsystem, buffer).andThen(
                                            CMD.shortWaitAndGrabSample(intakeSubsystem)
                                    )
                            )
                    )
                )
        );
    }

    @Override
    public void initialize() {
        fullSequence.initialize();
    }

    @Override
    public void execute() {
        fullSequence.execute();

        if (fullSequence.isFinished()) {
            this.done = intakeSubsystem.hasIntakeGotSample();

            if (!this.done) {
                fullSequence.end(false);
                buffer.reset();
                fullSequence.initialize();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (this.done) fullSequence.end(false);
        return this.done;
    }
}
