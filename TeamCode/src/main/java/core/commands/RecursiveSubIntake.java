package core.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import core.computerVision.Limelight;
import core.hardware.IndicatorLight;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class RecursiveSubIntake extends SelectCommand {
    public RecursiveSubIntake(
            Follower follower,
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Telemetry telemetry,
            IndicatorLight light
    ) {
        super(new HashMap<Object, Command>() {{
            put(false, CMD.grabSampleForSubCycles(
                    follower,
                    intakeSubsystem,
                    outtakeSubsystem,
                    limelight,
                    buffer,
                    telemetry,
                    light
            ));
            put(true, CMD.goToBasketForSubCycles(
                    follower,
                    intakeSubsystem,
                    outtakeSubsystem
            ));
        }},
        intakeSubsystem::hasIntakeGotSample);
    }
}
