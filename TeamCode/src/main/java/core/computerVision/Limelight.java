package core.computerVision;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.hardware.CachedServo;
import core.math.Vector;
import core.parameters.HardwareParameters;
import core.parameters.PositionalBounds;
import core.subsystems.Intake;

import java.util.AbstractMap.SimpleEntry;

public class Limelight {
    private final Limelight3A hardware;
    private final CachedServo arm;

    private double position = 0;

    public enum Targets {
        YellowOnly,
        RedAndYellow,
        BlueAndYellow,
        RedOnly,
        BlueOnly,
        BlueSideYellow,

        HotelYellow
    }

    public Limelight(HardwareMap hwmp, Targets targets) {
        this.hardware = hwmp.get(Limelight3A.class, "limelight");
        this.setTarget(targets);
        this.hardware.updatePythonInputs(0, 0, 0, 0, 0, 0, 0, 0);
        this.arm = new CachedServo(hwmp, "limelightServo");
        this.hide();
    }

    public void setTarget(Targets targets) {
        switch (targets) {
            case YellowOnly:
                this.hardware.pipelineSwitch(1);
                break;
            case RedAndYellow:
                this.hardware.pipelineSwitch(2);
                break;
            case BlueAndYellow:
                this.hardware.pipelineSwitch(3);
                break;
            case RedOnly:
                this.hardware.pipelineSwitch(4);
                break;
            case BlueOnly:
                this.hardware.pipelineSwitch(5);
                break;
            case HotelYellow:
                this.hardware.pipelineSwitch(6);
                break;
            case BlueSideYellow:
                this.hardware.pipelineSwitch(7);
                break;
        }
    }

    public void enable() {
        hardware.start();
    }
    public void logStatus(Telemetry telemetry) {
        telemetry.addData("LL3A STATUS", hardware.getStatus().toString());
    }
    public void disable() { hardware.stop(); }
    public void shutdown() { hardware.shutdown(); }

    public void load(int id) {
        this.hardware.pipelineSwitch(id);
    }

    public static class SamplePair {
        public SampleState optimal;
        public SampleState cached;

        public SamplePair(SampleState optimal, SampleState cached) {
            this.optimal = optimal;
            this.cached = cached;
        }
    }

    public static class SampleState {
        public double angle;
        public Vector center;

        public Vector robotPosition;
        public double robotRotation;

        public double slidePosition;
        public double intakeTilt;

        public SampleState(double angle, Vector center, Vector robotPosition, double robotRotation, double slidePosition, double intakeTilt) {
            this.angle = angle;
            this.center = center;
            this.robotPosition = robotPosition;
            this.robotRotation = robotRotation;
            this.slidePosition = slidePosition;
            this.intakeTilt = intakeTilt;
        }

        public SampleState() {
            this.angle = 0;
            this.center = Vector.cartesian(0, 0);
            this.robotPosition = Vector.cartesian(0, 0);
            this.robotRotation = 0;
            this.slidePosition = 0;
        }

        public void reset() {
            this.angle = 0;
            this.center = Vector.cartesian(0, 0);
            this.robotPosition = Vector.cartesian(0, 0);
            this.robotRotation = 0;
            this.slidePosition = 0;
        }
    }

    public SampleState query(Telemetry telemetry, Follower follower, Intake intakeSubsystem) {
        LLResult result = hardware.getLatestResult();

        telemetry.addData("SOMERESULT", result == null);
        if (result == null) return null;

        double[] result_array = result.getPythonOutput();
        telemetry.addData("SOMEPYTHON", result_array == null);

        if (result_array == null) return null;
        if (result_array.length == 0) return null;
        double angle = result_array[0];

        // This is POSSIBLE, but so unlikely it can be treated as an error
        if (angle == 0) return null;

        Vector center = Vector.cartesian(result_array[1], result_array[2]);
        Pose current = follower.getPose();
        return new SampleState(angle, center, Vector.cartesian(current.getX(),
                current.getY()), current.getHeading(), intakeSubsystem.getSlidePosition(),
                intakeSubsystem.getTilt());
    }

    public SamplePair query_two(Telemetry telemetry, Follower follower, Intake intakeSubsystem) {
        LLResult result = hardware.getLatestResult();

        telemetry.addData("SOMERESULT", result == null);
        if (result == null) return null;

        double[] result_array = result.getPythonOutput();
        telemetry.addData("SOMEPYTHON", result_array == null);

        if (result_array == null) return null;
        if (result_array.length == 0) return null;
        double angle = result_array[0];
        double angle2 = result_array[3];

        // This is POSSIBLE, but so unlikely it can be treated as an error
        if (angle == 0) return null;

        Vector center = Vector.cartesian(result_array[1], result_array[2]);
        Vector center2 = Vector.cartesian(result_array[4], result_array[5]);
        Pose current = follower.getPose();

        SampleState optimal = new SampleState(angle, center, Vector.cartesian(current.getX(),
                current.getY()), current.getHeading(), intakeSubsystem.getSlidePosition(),
                intakeSubsystem.getTilt());

        if (angle2 == 0) return new SamplePair(optimal, null);

        SampleState cached = new SampleState(angle2, center2, Vector.cartesian(current.getX(),
                current.getY()), current.getHeading(), intakeSubsystem.getSlidePosition(),
                intakeSubsystem.getTilt());

        return new SamplePair(optimal, cached);
    }

    public void raise() { this.arm.setPosition(PositionalBounds.ServoPositions.limelightUp); }
    public void hide() { this.arm.setPosition(PositionalBounds.ServoPositions.limelightDown); }

    public boolean isRaised() {
        return this.arm.elapsedTime() > 0.3;
    }

    public boolean isHidden() {
        return this.arm.elapsedTime() > 0.3;
    }
}
