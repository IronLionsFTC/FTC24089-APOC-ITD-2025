package core.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.computerVision.Limelight;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class GrabSampleAbortIfEmpty extends ConditionalCommand {

    public GrabSampleAbortIfEmpty(
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Telemetry telemetry,
            Follower follower
    ) {
        super(
                CMD.resetCV(buffer),

                new SequentialCommandGroup(
                        CMD.retractIntakeSlightly(intakeSubsystem),
                        CMD.sleep(100),
                        CMD.jerkAndStartRepeating(
                                intakeSubsystem,
                                outtakeSubsystem,
                                limelight,
                                buffer,
                                telemetry,
                                follower
                        )
                ),

                intakeSubsystem::hasIntakeGotSample
        );
    }
}