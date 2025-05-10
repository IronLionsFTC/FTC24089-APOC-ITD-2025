package core.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import core.computerVision.Limelight;
import core.state.Subsystems;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class GrabSampleAbortIfEmpty extends SelectCommand {

    private Intake intakeSubsystem;

    public GrabSampleAbortIfEmpty(
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Telemetry telemetry,
            Follower follower
    ) {
        super(new HashMap<Object, Command>() {{
            put(
                    true,
                    CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
            );
            put(
                    false,
                    CMD.retractIntake(intakeSubsystem).alongWith(CMD.raiseLimelight(limelight)).andThen(
                            CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false)
                    ).andThen(
                            CMD.driveToSampleUseSlides(follower, intakeSubsystem, buffer, telemetry)
                    ).andThen(
                            CMD.sleep(300)
                    ).andThen(
                            CMD.grabSample(intakeSubsystem)
                    ).andThen(
                            CMD.sleep(100)
                    ).andThen(
                            CMD.grabSampleAbortIfEmpty(intakeSubsystem, outtakeSubsystem, limelight, buffer, telemetry, follower)
                    )
            );
        }}, intakeSubsystem::hasIntakeGotSample);
    }
}