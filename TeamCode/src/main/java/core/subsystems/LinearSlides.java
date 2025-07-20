package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

import core.hardware.CachedMotor;
import core.hardware.MasterSlaveMotorPair;
import core.math.Utility;

public class LinearSlides extends SubsystemBase {

    // PID controller setup, allows for tuning.
    // This is just a reference to an owned PIDController specific to the parent system,
    // for example the intake subsystem is static and owns a pidcontroller which it can update,
    // the changes simply propagate down to the dynamic LinearSlide subsystem
    private PIDController leftController;
    private PIDController rightController;

    // In some situations, allow for different feedforward depending on direction.
    // For example, generally avoid having vertical slides have a negative feedforward.
    private double positiveFeedForward;
    private double negativeFeedForward;

    // Option to completely disable the entire subsystem
    private boolean enabled = true;
    // When retracting slides, if they are within this threshold (0-1), then cut power
    private double cutPowerOnNegativeThreshold = 0.005;
    private double maximumExtension;
    private boolean powerOnRetraction = true;
    private double target = 0;
    private boolean disabled;

    private CachedMotor leftMotor;
    private CachedMotor rightMotor;
    private Telemetry telemetry;
    private VoltageSensor voltageSensor;
    private BooleanSupplier forceDown;
    private BooleanSupplier zeroing;

    // Allow for different negative and positive feedforward values on construction
    public LinearSlides(
            CachedMotor leftMotor,
            CachedMotor rightMotor,
            PIDController leftController,
            PIDController rightController,
            Telemetry telemetry,
            VoltageSensor voltageSensor,
            double positiveFeedForward,
            double negativeFeedForward,
            double maximumExtension,
            BooleanSupplier forceDown,
            BooleanSupplier zeroing
    ) {
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.leftController = leftController;
        this.rightController = rightController;
        this.positiveFeedForward = positiveFeedForward;
        this.negativeFeedForward = negativeFeedForward;
        this.maximumExtension = maximumExtension;
        this.forceDown = forceDown;
        this.zeroing = zeroing;
        this.disabled = false;
    }

    public void setTarget(double relative) {
        this.target = Utility.clamp(relative, 0, 2);
    }
    public void zeroEncoder() {
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }

    // Inherit the periodic update function from the subsystem base, this sets the motor powers from the PID
    @Override
    public void periodic() {

        double leftResponse = this.leftController.calculate(this.leftMotor.getPosition(), this.target * this.maximumExtension);
        double rightResponse = this.rightController.calculate(this.rightMotor.getPosition(), this.target * this.maximumExtension);

        double leftFeedforward;
        double rightFeedforward;

        if (leftResponse > 0) { leftFeedforward = positiveFeedForward; }
        else { leftFeedforward = -negativeFeedForward; }
        if (rightResponse > 0) { rightFeedforward = positiveFeedForward; }
        else { rightFeedforward = -negativeFeedForward; }

        // If the slides are at the return position, cut power
        if (this.leftMotor.getPosition() < this.cutPowerOnNegativeThreshold * this.maximumExtension && target < cutPowerOnNegativeThreshold) {
            leftResponse = 0;
        } else {
            // Apply calculated feedforward
            leftResponse += leftFeedforward;
        }
        if (this.rightMotor.getPosition() < this.cutPowerOnNegativeThreshold * this.maximumExtension && target < cutPowerOnNegativeThreshold) {
            rightResponse = 0;
        } else {
            // Apply calculated feedforward
            rightResponse += rightFeedforward;
        }

        boolean needsForce = false;

        if (this.target == 0 && getRelative() < 0.2) {
            if (this.forceDown.getAsBoolean()) {
                leftResponse = -0.8;
                rightResponse = -0.8;
                needsForce = true;
                this.zeroEncoder();
            }
        }

        if (this.zeroing.getAsBoolean()) {
            leftResponse = -1;
            rightResponse = -1;
            needsForce = true;

            this.zeroEncoder();
        }

        if ((!this.disabled) || needsForce) {

            // Set the master-slave paradigm to use the power
            leftMotor.setPower(leftResponse);
            rightMotor.setPower(rightResponse);
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    // Position in range 0 to 1
    public double getRelative() {
        return this.getPosition() / this.maximumExtension;
    }

    public void setFeedForward(double feedForward) { this.positiveFeedForward = feedForward; }
    public void setFeedBackward(double feedBackward) { this.negativeFeedForward = feedBackward; }
    public void setZeroPowerOnRetraction() { this.powerOnRetraction = false; }
    public boolean atTarget() {
        return Math.abs(this.getRelative() - this.target) < 0.1;
    }
    public boolean nearlyAtTarget() {
        return Math.abs(this.getRelative() - this.target) < 0.3;
    }

    public void enable() {
        this.disabled = false;
    }

    public void disable() {
        this.disabled = true;
    }
    public double getPosition() {
        return (this.leftMotor.getPosition() + this.rightMotor.getPosition()) / 2.0;
    }

    public boolean isDisabled() {
        return disabled;
    }

    public boolean isForcedDown() {
        return this.forceDown.getAsBoolean();
    }
}
