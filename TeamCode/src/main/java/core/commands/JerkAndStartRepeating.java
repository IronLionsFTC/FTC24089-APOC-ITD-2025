package core.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.computerVision.Limelight;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class JerkAndStartRepeating extends ConditionalCommand {

    public JerkAndStartRepeating(
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Telemetry telemetry,
            Follower follower
    ) {
        super(
                CMD.resetCV(buffer),

                CMD.retryAndRepeat(
                        intakeSubsystem,
                        outtakeSubsystem,
                        limelight,
                        buffer,
                        telemetry,
                        follower
                ),

                intakeSubsystem::hasIntakeGotSample
        );
    }
}
