package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

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
    private double cutPowerOnNegativeThreshold = 0.05;
    private double maximumExtension;
    private double target = 0;

    private MasterSlaveMotorPair motors;

    // Public constructor requires non-null motor pair to avoid overhead
    public LinearSlides(MasterSlaveMotorPair motors, PIDController pidController, double feedForward, double maximumExtension) {
        this.motors = motors;
        this.pidController = pidController;
        this.positiveFeedForward = feedForward;
        this.negativeFeedForward = feedForward;
        this.maximumExtension = maximumExtension;
    }

    // Allow for different negative and positive feedforward values on construction
    public LinearSlides(MasterSlaveMotorPair motors,
            PIDController pidController,
            double positiveFeedForward,
            double negativeFeedForward,
            double maximumExtension) {
        this.motors = motors;
        this.pidController = pidController;
        this.positiveFeedForward = positiveFeedForward;
        this.negativeFeedForward = negativeFeedForward;
        this.maximumExtension = maximumExtension;
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
        if (response < this.cutPowerOnNegativeThreshold) {
            response = 0;
        } else {
            // Apply calculated feedforward
            response += feedforward;
        }

        // Set the master-slave paradigm to use the power
        motors.setPower(response);
    }
}
