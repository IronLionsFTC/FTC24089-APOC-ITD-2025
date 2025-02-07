package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import core.hardware.CachedEncoder;
import core.hardware.CachedMotor;
import core.math.Vector;
import core.parameters.RobotParameters;

public class Drivebase extends SubsystemBase {

    // Parameters to control the drivetrain
    public Vector driveVector;
    public double driveScalar;
    private double yaw;

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

    // Drivetrain motors
    private CachedMotor frontRight;
    private CachedMotor frontLeft;
    private CachedMotor backLeft;
    private CachedMotor backRight;

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
            this.left = new CachedEncoder(hwmp, RobotParameters.Odometry.HardwareMapNames.left);
            this.right = new CachedEncoder(hwmp, RobotParameters.Odometry.HardwareMapNames.right);
            this.sideways = new CachedEncoder(hwmp, RobotParameters.Odometry.HardwareMapNames.sideways);
        }

        // Derive the yaw from the encoder positions
        public double calculateYaw() {
            return (this.left.read() + this.right.read()) * this.scalar;
        }

        // Reset the dt odometry
        public void zero() {
            this.left.reset();
            this.right.reset();
            this.sideways.reset();
        }
    }

    public Drivebase(HardwareMap hwmp) {
        this.driveVector = Vector.cartesian(0, 0);
        this.driveScalar = 0;

        // Create Motors
        this.frontRight = new CachedMotor(hwmp, RobotParameters.Motors.HardwareMapNames.frontRight);
        this.frontLeft = new CachedMotor(hwmp, RobotParameters.Motors.HardwareMapNames.frontLeft);
        this.backRight = new CachedMotor(hwmp, RobotParameters.Motors.HardwareMapNames.backRight);
        this.backLeft = new CachedMotor(hwmp, RobotParameters.Motors.HardwareMapNames.backLeft);

        // Set reversed motors
        this.frontRight.setReversed(RobotParameters.Motors.Reversed.frontRight);
        this.frontLeft.setReversed(RobotParameters.Motors.Reversed.frontLeft);
        this.backRight.setReversed(RobotParameters.Motors.Reversed.backRight);
        this.backLeft.setReversed(RobotParameters.Motors.Reversed.backLeft);

        // Apply zero power braking
        this.frontRight.setZeroPowerBehaviour(RobotParameters.Motors.ZeroPowerModes.drivetrain);
        this.frontLeft.setZeroPowerBehaviour(RobotParameters.Motors.ZeroPowerModes.drivetrain);
        this.backRight.setZeroPowerBehaviour(RobotParameters.Motors.ZeroPowerModes.drivetrain);
        this.backLeft.setZeroPowerBehaviour(RobotParameters.Motors.ZeroPowerModes.drivetrain);

        // Odometry
        this.odometry = new Odometry(hwmp);
    }

    @Override
    public void periodic() {

        // Calculate the yaw
        this.yaw = this.odometry.calculateYaw();

        this.frontLeft.setPower((driveVector.x - driveVector.y) * driveScalar + r);
        this.frontRight.setPower((-driveVector.x - driveVector.y) * driveScalar - r);
        this.backLeft.setPower((-driveVector.x - driveVector.y) * driveScalar + r);
        this.backRight.setPower((driveVector.x - driveVector.y) * driveScalar - r);
    }
}
