package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.computerVision.Limelight;
import core.subsystems.Intake;

public class ScanForTwoSamples extends CommandBase {
    private Limelight limelight;

    private Limelight.SampleState result;
    private Limelight.SampleState cache;

    private Telemetry telemetry;

    private Follower follower;
    private Intake intakeSubsystem;

    private boolean check;
    private double tilt = 0;

    public ScanForTwoSamples(
            Limelight limelight, Limelight.SampleState buffer, Limelight.SampleState cache, Telemetry telemetry, Follower follower, Intake intakeSubsystem, boolean isSub
    ) {
        this.limelight = limelight;
        this.result = buffer;
        this.cache = cache;
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
        Limelight.SamplePair detection = limelight.query_two(telemetry, follower, intakeSubsystem);

        if (detection != null) {
            this.result.angle = detection.optimal.angle;
            this.result.center = detection.optimal.center;
            this.result.robotPosition = detection.optimal.robotPosition;
            this.result.robotRotation = detection.optimal.robotRotation;
            this.result.slidePosition = intakeSubsystem.getSlidePosition();

            if (detection.cached != null) {
                this.cache.angle = detection.cached.angle;
                this.cache.center = detection.cached.center;
                this.cache.robotPosition = detection.cached.robotPosition;
                this.cache.robotRotation = detection.cached.robotRotation;
                this.cache.slidePosition = intakeSubsystem.getSlidePosition();
            }
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
        this.intakeSubsystem.setTilt(0);
    }
}
