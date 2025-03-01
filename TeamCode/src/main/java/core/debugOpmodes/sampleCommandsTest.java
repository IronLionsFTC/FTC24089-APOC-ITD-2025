package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import core.commands.CMD;
import core.subsystems.Intake;
import core.subsystems.Outtake;

public class sampleCommandsTest extends CommandOpMode {
    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry);

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);

        schedule(
            new SequentialCommandGroup(
                    CMD.sleepUntil(this::opModeIsActive),
                    new RunCommand(telemetry::update),

                    CMD.extendIntake(intakeSubsystem, 45),
                    CMD.grabSample(intakeSubsystem),
                    CMD.retractIntakeAndTransfer(intakeSubsystem, outtakeSubsystem)
            )
        );
    }
}
