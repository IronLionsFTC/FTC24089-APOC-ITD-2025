package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.AbstractMap;

import core.computerVision.Limelight;
import core.subsystems.Intake;

public class ScanForTwoSamples extends CommandBase {
    private Limelight limelight;
    private Limelight.SampleState result;
    private Limelight.SampleState result2;
    private Telemetry telemetry;

    private Follower follower;
    private Intake intakeSubsystem;

    private boolean check;
    private double tilt = 0;

    public ScanForTwoSamples(Limelight limelight,
                             Limelight.SampleState buffer1,
                             Limelight.SampleState buffer2,
                             Telemetry telemetry, Follower follower, Intake intakeSubsystem, boolean isSub) {
        this.limelight = limelight;
        this.result = buffer1;
        this.result2 = buffer2;
        this.telemetry = telemetry;
        this.follower = follower;
        this.intakeSubsystem = intakeSubsystem;
        this.check = isSub;
    }

    public ScanForTwoSamples tilt(double newTilt) {
        this.tilt = newTilt;
        return this;
    }

    @Override
    public void initialize() {
        this.limelight.enable();
        this.intakeSubsystem.setTilt(this.tilt);
    }

    @Override
    public void execute() {
        this.limelight.logStatus(telemetry);
        AbstractMap.SimpleEntry<Limelight.SampleState, Limelight.SampleState> detection = limelight.query_two(telemetry, follower, intakeSubsystem);

        if (detection != null) {
            Limelight.SampleState detectionA = detection.getKey();
            Limelight.SampleState detectionB = detection.getValue();

            double x = detectionA.robotPosition.x;
            double y = detectionA.robotPosition.y;
            double r = detectionA.robotRotation;

            double tx = 1 - 0.8 * detectionA.center.x;
            double ty = 2.7 - Math.pow(detectionA.center.y, 2);

            double relativeX = ty * Math.cos(r) + tx * Math.cos(r - Math.toRadians(90));
            double relativeY = ty * Math.sin(r) + tx * Math.sin(r - Math.toRadians(90));

            double targetX = x + relativeX;
            double targetY = y + relativeY;

            if (targetY < -16 && check) return;
            if (targetX < 49 && check) return;

            telemetry.addData("ANGLE", detectionA.angle);
            this.result.angle = detectionA.angle;
            this.result.center = detectionA.center;
            this.result.robotPosition = detectionA.robotPosition;
            this.result.robotRotation = detectionA.robotRotation;
            this.result.slidePosition = intakeSubsystem.getSlideExtension();
            this.result.slideOffset = intakeSubsystem.getOffset();

            double x2 = detectionB.robotPosition.x;
            double y2 = detectionB.robotPosition.y;
            double r2 = detectionB.robotRotation;

            double tx2 = 1 - 0.8 * detectionB.center.x;
            double ty2 = 2.7 - Math.pow(detectionB.center.y, 2);

            double relativeX2 = ty2 * Math.cos(r) + tx2 * Math.cos(r2 - Math.toRadians(90));
            double relativeY2 = ty2 * Math.sin(r) + tx2 * Math.sin(r2 - Math.toRadians(90));

            double targetX2 = x2 + relativeX2;
            double targetY2 = y2 + relativeY2;

            if (targetY2 < -16 && check) return;
            if (targetX2 < 49 && check) return;

            telemetry.addData("ANGLE", detectionB.angle);
            this.result2.angle = detectionB.angle;
            this.result2.center = detectionB.center;
            this.result2.robotPosition = detectionB.robotPosition;
            this.result2.robotRotation = detectionB.robotRotation;
            this.result2.slidePosition = intakeSubsystem.getSlideExtension();
            this.result2.slideOffset = intakeSubsystem.getOffset();
        }
        else telemetry.addLine("IS NULL");
    }

    @Override
    public boolean isFinished() {
        telemetry.addData("ANGLE2", result.angle);
        return this.result.angle != 0 && this.result2.angle != 0;
    }

    @Override
    public void end(boolean i) {
        this.limelight.disable();
        this.intakeSubsystem.setTilt(0);
    }
}
