package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.computerVision.Limelight;
import core.subsystems.Intake;

public class ScanForSample extends CommandBase {
    private Limelight limelight;

    private Limelight.SampleState result;

    private Telemetry telemetry;

    private Follower follower;
    private Intake intakeSubsystem;

    private boolean check;
    private double tilt = 0;

    public ScanForSample(
            Limelight limelight, Limelight.SampleState buffer, Telemetry telemetry, Follower follower, Intake intakeSubsystem, boolean isSub
    ) {
        this.limelight = limelight;
        this.result = buffer;
        this.telemetry = telemetry;
        this.follower = follower;
        this.intakeSubsystem = intakeSubsystem;
        this.check = isSub;
    }

    public ScanForSample tilt(double newTilt) {
        this.tilt = newTilt;
        return this;
    }

    @Override
    public void initialize() {
        this.limelight.enable();
        this.intakeSubsystem.setTilt(this.tilt);
        this.result.reset();
    }

    @Override
    public void execute() {
        this.limelight.logStatus(telemetry);
        Limelight.SampleState detection = limelight.query(telemetry, follower, intakeSubsystem);

        if (detection != null) {

            double x = detection.robotPosition.x;
            double y = detection.robotPosition.y;
            double r = detection.robotRotation;

            double tx = 1 - 0.8 * detection.center.x;
            double ty = 2.7 - Math.pow(detection.center.y, 2);

            double relativeX = ty * Math.cos(r) + tx * Math.cos(r - Math.toRadians(90));
            double relativeY = ty * Math.sin(r) + tx * Math.sin(r - Math.toRadians(90));

            double targetX = x + relativeX;
            double targetY = y + relativeY;

            if (targetY < -16 && check) return;
            if (targetX < 49 && check) return;

            telemetry.addData("ANGLE", detection.angle);
            this.result.angle = detection.angle;
            this.result.center = detection.center;
            this.result.robotPosition = detection.robotPosition;
            this.result.robotRotation = detection.robotRotation;
            this.result.slidePosition = intakeSubsystem.getSlidePosition();
        }
        else telemetry.addLine("IS NULL");
    }

    @Override
    public boolean isFinished() {
        telemetry.addData("ANGLE2", result.angle);
        return this.result.angle != 0 && this.result.angle != 90;
    }

    @Override
    public void end(boolean i) {
        this.limelight.disable();
    }
}
