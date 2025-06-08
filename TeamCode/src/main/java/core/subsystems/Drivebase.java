package core.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.hardware.CachedEncoder;
import core.hardware.CachedMotor;
import core.math.Vector;
import core.parameters.HardwareParameters;
import core.parameters.pidfCoefficients;

public class Drivebase extends SubsystemBase {

    // Parameters to control the drivetrain
    private Vector driveVector;
    private double yawInput;
    private double driveScalar;
    private double yaw = 0;
    private boolean active;

    // This is the yaw value to HOLD until either:
    //      the user presses BUMPERS for -45/45 change
    //      the user moves the right joystick on x axis, manually
    //          powering the wheels and disabling yaw correction
    //          until they let go
    //  this should NOT be mutated from outside of this class,
    //      and is only exposed in case
    public double targetYaw = 0;
    // This value stores whether or not the user is overriding yaw control,
    //      and thus whether the yaw correction should be disabled until they let go
    private boolean lastYawActionWasManual = false;
    private PIDFController yawCorrectionController;

    // Drivetrain motors
    private CachedMotor frontRight;
    private CachedMotor frontLeft;
    private CachedMotor backLeft;
    private CachedMotor backRight;

    // Telemetry
    private Telemetry telemetry;

    // Odometry
    public Odometry odometry;

    public static class Odometry {

        // Scale the encoder ticks into degrees
        private static final double scalar = 0.00925;

        // Encoders for YAW tracking
        private CachedEncoder left;
        private CachedEncoder right;
        private CachedEncoder sideways;

        public Odometry(HardwareMap hwmp) {

            // Initialise Encoders
            this.left = new CachedEncoder(hwmp, HardwareParameters.Odometry.HardwareMapNames.left);
            this.right = new CachedEncoder(hwmp, HardwareParameters.Odometry.HardwareMapNames.right);
            this.sideways = new CachedEncoder(hwmp, HardwareParameters.Odometry.HardwareMapNames.sideways);
            this.zero();
        }

        // Derive the yaw from the encoder positions
        public double calculateYaw() {
            return (this.left.read() + this.right.read()) * scalar;
        }

        // Reset the dt odometry
        public void zero() {
            this.left.reset();
            this.right.reset();
            this.sideways.reset();
        }
    }

    public Drivebase(HardwareMap hwmp, Telemetry telemetry) {
        this.telemetry = telemetry;

        // PID
        this.yawCorrectionController = new PIDFController(
                pidfCoefficients.Drivetrain.p,
                pidfCoefficients.Drivetrain.i,
                pidfCoefficients.Drivetrain.d,
                pidfCoefficients.Drivetrain.f
        );

        this.driveVector = Vector.cartesian(0, 0);
        this.driveScalar = 1;

        // Create Motors
        this.frontRight = new CachedMotor(hwmp, HardwareParameters.Motors.HardwareMapNames.frontRight);
        this.frontLeft = new CachedMotor(hwmp, HardwareParameters.Motors.HardwareMapNames.frontLeft);
        this.backRight = new CachedMotor(hwmp, HardwareParameters.Motors.HardwareMapNames.backRight);
        this.backLeft = new CachedMotor(hwmp, HardwareParameters.Motors.HardwareMapNames.backLeft);

        // Set reversed motors
        this.frontRight.setReversed(HardwareParameters.Motors.Reversed.frontRight);
        this.frontLeft.setReversed(HardwareParameters.Motors.Reversed.frontLeft);
        this.backRight.setReversed(HardwareParameters.Motors.Reversed.backRight);
        this.backLeft.setReversed(HardwareParameters.Motors.Reversed.backLeft);

        // Apply zero power braking
        this.frontRight.setZeroPowerBehaviour(HardwareParameters.Motors.ZeroPowerModes.drivetrain);
        this.frontLeft.setZeroPowerBehaviour(HardwareParameters.Motors.ZeroPowerModes.drivetrain);
        this.backRight.setZeroPowerBehaviour(HardwareParameters.Motors.ZeroPowerModes.drivetrain);
        this.backLeft.setZeroPowerBehaviour(HardwareParameters.Motors.ZeroPowerModes.drivetrain);

        // Odometry
        this.odometry = new Odometry(hwmp);
        this.active = true;
    }

    public void setYaw(double newYaw) {
        this.targetYaw = newYaw;
    }

    public void rotate45DegreesCCW() {
        this.setYaw(this.targetYaw - 45);
        this.lastYawActionWasManual = false;
    }

    public void rotate45DegreesCW() {
        this.setYaw(this.targetYaw + 45);
        this.lastYawActionWasManual = false;
    }

    @Override
    public void periodic() {

        if (!active) {
            this.frontLeft.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.FLOAT);
            this.frontRight.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.FLOAT);
            this.backRight.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.FLOAT);
            this.backLeft.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.FLOAT);
        } else {
            this.frontLeft.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.BRAKE);
            this.backLeft.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.BRAKE);
        }

        this.lastYawActionWasManual = this.yawInput != 0;

        // If the pid coefficients are being tuned, update them constantly
        telemetry.addData("Drivebase tuning: ", pidfCoefficients.Drivetrain.tuning);
        if (pidfCoefficients.Drivetrain.tuning) {
            this.yawCorrectionController.setPIDF(
                    pidfCoefficients.Drivetrain.p,
                    pidfCoefficients.Drivetrain.i,
                    pidfCoefficients.Drivetrain.d,
                    pidfCoefficients.Drivetrain.f
            );
        }

        // Calculate the yaw
        this.yaw = -this.odometry.calculateYaw();
        telemetry.addData("YAW", this.yaw);

        // PID (position, target)
        double r = this.yawInput;

        if (this.lastYawActionWasManual) { this.targetYaw = this.yaw; }
        else { r = this.yawCorrectionController.calculate(this.yaw, this.targetYaw); }

        if (Math.abs(r) < 0.1) r = 0;

        this.frontLeft.setPower((driveVector.x - driveVector.y) * driveScalar + r);
        this.frontRight.setPower((-driveVector.x - driveVector.y) * driveScalar - r);
        this.backLeft.setPower((-driveVector.x - driveVector.y) * driveScalar + r);
        this.backRight.setPower((driveVector.x - driveVector.y) * driveScalar - r);
    }

    public void setDriveVector(Vector driveVector) {
        this.driveVector = driveVector;
    }

    public void setDriveScalar(double driveScalar) {
        this.driveScalar = driveScalar;
    }

    public void stopDrivebase() {
        this.driveVector = Vector.cartesian(0, 0);
    }

    public void resetSpeed() {
        this.driveScalar = 1;
    }

    public void setYawInput(double yawInput) {
        this.yawInput = yawInput;
    }

    public void enable() {
        this.active = true;
    }

    public void disable() {
        this.active = false;
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
        this.backLeft.setPower(0);
        this.backRight.setPower(0);
    }

    public boolean active(){
        return this.active;
    }
}
