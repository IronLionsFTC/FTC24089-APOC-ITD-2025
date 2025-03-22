package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.computerVision.Limelight;

public class ScanForSample extends CommandBase {
    private Limelight limelight;
    private Limelight.SampleState result;
    private Telemetry telemetry;

    public ScanForSample(Limelight limelight, Limelight.SampleState buffer, Telemetry telemetry) {
        this.limelight = limelight;
        this.result = buffer;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        this.limelight.enable();
    }

    @Override
    public void execute() {
        this.limelight.logStatus(telemetry);
        Limelight.SampleState detection = limelight.query(telemetry);

        if (detection != null) {
            telemetry.addData("ANGLE", detection.angle);
            this.result.angle = detection.angle;
            this.result.center = detection.center;
        }
        else telemetry.addLine("IS NULL");
    }

    @Override
    public boolean isFinished() {
        telemetry.addData("ANGLE2", result.angle);
        return this.result.angle != 0;
    }

    @Override
    public void end(boolean i) {
        this.limelight.disable();
    }
}
