package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

import core.hardware.MasterSlaveMotorPair;
import core.math.Utility;

public class LinearSlides extends SubsystemBase {

    // PID controller setup, allows for tuning.
    // This is just a reference to an owned PIDController specific to the parent system,
    // for example the intake subsystem is static and owns a pidcontroller which it can update,
    // the changes simply propagate down to the dynamic LinearSlide subsystem
    private PIDController pidController;

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

    private MasterSlaveMotorPair motors;
    private Telemetry telemetry;
    private VoltageSensor voltageSensor;
    private BooleanSupplier forceDown;
    private BooleanSupplier zeroing;

    // Allow for different negative and positive feedforward values on construction
    public LinearSlides(
            MasterSlaveMotorPair motors,
            PIDController pidController,
            Telemetry telemetry,
            VoltageSensor voltageSensor,
            double positiveFeedForward,
            double negativeFeedForward,
            double maximumExtension,
            BooleanSupplier forceDown
            ) {
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;
        this.motors = motors;
        this.pidController = pidController;
        this.positiveFeedForward = positiveFeedForward;
        this.negativeFeedForward = negativeFeedForward;
        this.maximumExtension = maximumExtension;
        this.forceDown = forceDown;
        this.zeroing = () -> false;
    }

    // Allow for different negative and positive feedforward values on construction
    public LinearSlides(
            MasterSlaveMotorPair motors,
            PIDController pidController,
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
        this.motors = motors;
        this.pidController = pidController;
        this.positiveFeedForward = positiveFeedForward;
        this.negativeFeedForward = negativeFeedForward;
        this.maximumExtension = maximumExtension;
        this.forceDown = forceDown;
        this.zeroing = zeroing;
    }

    public void setTarget(double relative) {
        this.target = Utility.clamp(relative, 0, 1);
    }

    public void zeroEncoder() {
        motors.resetEncoder();
    }

    // Inherit the periodic update function from the subsystem base, this sets the motor powers from the PID
    @Override
    public void periodic() {
        double response = this.pidController.calculate(this.motors.getPosition(), this.target * this.maximumExtension);
        double feedforward;
        if (response > 0) { feedforward = positiveFeedForward; }
        else { feedforward = -negativeFeedForward; }

        // If the slides are at the return position, cut power
        if (this.motors.getPosition() < this.cutPowerOnNegativeThreshold * this.maximumExtension && target < cutPowerOnNegativeThreshold) {
            // response = 0;
        } else {
            // Apply calculated feedforward
            response += feedforward;
        }

        if (this.target == 0 && getRelative() < 0.07) {
            if (this.forceDown.getAsBoolean()) {
                response = -0.4;
            }
        }

        if (this.zeroing.getAsBoolean()) {
            response = -0.5;
            this.motors.resetEncoder();
        }

        // Set the master-slave paradigm to use the power
        motors.setPower(response);
    }

    // Position in range 0 to 1
    public double getRelative() {
        return this.motors.getPosition() / this.maximumExtension;
    }

    public void setFeedForward(double feedForward) { this.positiveFeedForward = feedForward; }
    public void setFeedBackward(double feedBackward) { this.negativeFeedForward = feedBackward; }
    public void setZeroPowerOnRetraction() { this.powerOnRetraction = false; }
    public boolean atTarget() {
        return Math.abs(this.getRelative() - this.target) < 0.1;
    }
}
