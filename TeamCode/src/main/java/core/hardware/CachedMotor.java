package core.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CachedMotor {
    private double power;
    private double position;
    private final MotorEx motor;

    public CachedMotor(HardwareMap hwmp, String name) {
        this.motor = new MotorEx(hwmp, name);
        this.motor.resetEncoder();
        this.power = 0;
        this.position = 0;
    }

    public boolean setPower(double power) {
        double error = Math.abs(power - this.power);
        if (error > 0.01 || power == 0) {
            this.power = power;
            this.motor.set(this.power);
            return true;
        }
        return false;
    }

    public double getPower() {
        return this.power;
    }

    public void stop() {
        this.power = 0;
        this.motor.set(this.power);
    }

    public void resetEncoder() {
        this.motor.resetEncoder();
    }

    // Update position and return
    public double getPosition() {
        this.position = this.motor.getCurrentPosition();
        return this.position;
    }

    // Get position as of last query
    public double getLastPosition() {
        return this.position;
    }

    public void setZeroPowerBehaviour(Motor.ZeroPowerBehavior zpb) {
        this.motor.setZeroPowerBehavior(zpb);
    }

    public void setReversed(boolean reversed) {
        this.motor.setInverted(reversed);
    }
}
