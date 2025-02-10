package core.hardware;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CachedServo {
    private Servo servo;
    private double position;
    private Timer timeSinceUpdate;

    public CachedServo(HardwareMap hwmp, String name) {
        this.servo = hwmp.get(Servo.class, name);
        this.position = 0.0;
    }

    public CachedServo(HardwareMap hwmp, String name, double position) {
        this.servo = hwmp.get(Servo.class, name);
        this.position = position;
        this.servo.setPosition(this.position);
    }

    public void setPosition(double position) {
        if (this.position != position) {
            this.position = position;
            servo.setPosition(this.position);
            timeSinceUpdate.resetTimer();
        }
    }

    public double getPosition() {
        return this.position;
    }

    public double secondsSinceMovement() {
        return timeSinceUpdate.getElapsedTimeSeconds();
    }

    public void resetTimer() {
        timeSinceUpdate.resetTimer();
    }
}