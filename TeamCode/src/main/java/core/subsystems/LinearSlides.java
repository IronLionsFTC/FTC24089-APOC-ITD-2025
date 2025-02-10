package core.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;

import core.hardware.MasterSlaveMotorPair;

public class LinearSlides {

    // PID controller setup, allows for tuning
    private PIDController pidController;

    // In some situations, allow for different feedforward depending on direction.
    // For example, generally avoid having vertical slides have a negative feedforward.
    private double positiveFeedForward;
    private double negativeFeedForward;

    private MasterSlaveMotorPair motors;

    // Public constructor requires non-null motor pair to avoid overhead
    public LinearSlides(MasterSlaveMotorPair motorPair, double feedForward) {
        this.motors = motorPair;
        this.positiveFeedForward = feedForward;
        this.negativeFeedForward = feedForward;
    }
}
