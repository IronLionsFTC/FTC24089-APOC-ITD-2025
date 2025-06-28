package core.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

import core.computerVision.Limelight;
import core.subsystems.Drivebase;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class TeleOpSpecButton extends ConditionalCommand {
    public TeleOpSpecButton(
            Intake intakeSubsystem,
            Outtake outtakeSubsystem,
            Limelight limelight,
            Limelight.SampleState buffer,
            Follower follower,
            Drivebase drivebase,
            Telemetry telemetry,
            BooleanSupplier interupt,
            Limelight.Targets targets
    ) {
        super(
                CMD.resetCV(buffer).andThen(
                        new InstantCommand(limelight::disable)
                ).andThen(
                        new InstantCommand(limelight::enable).andThen(
                                CMD.target(limelight, targets)
                        )
                ).andThen(
                        new IronLionsInterrupt(
                                CMD.disableDrivebase(drivebase).andThen(
                                        CMD.scanForSample(limelight, buffer, telemetry, follower, intakeSubsystem, false).andThen(
                                                (CMD.driveToSampleUseSlidesSpec(follower, intakeSubsystem, buffer, telemetry).alongWith(
                                                        CMD.alignClaw(intakeSubsystem, buffer)
                                                )).andThen(
                                                        CMD.resetCV(buffer)
                                                ).andThen(
                                                        CMD.shortWaitAndGrabSample(intakeSubsystem)
                                                )
                                        )
                                ),
                                interupt).andThen(
                                CMD.resetCV(buffer)
                        ).andThen(
                                new InstantCommand(limelight::disable)
                        ).andThen(
                                CMD.enableDrivebase(drivebase)
                        )
                ),

                CMD.retractIntakeAndTransferHalf(intakeSubsystem, outtakeSubsystem),

                intakeSubsystem::isSlidesRetracted
        );
    }
}
