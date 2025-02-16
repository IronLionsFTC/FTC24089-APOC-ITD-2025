package core.hardware;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CachedServo {
    private final Servo servo;
    private double position;
    private boolean inverse;
    private final Timer timeSinceUpdate = new Timer();

    // Expose a constructor pulling a Servo of name from hardwaremap
    public CachedServo(HardwareMap hwmp, String name) {
        this.servo = hwmp.get(Servo.class, name);
        this.inverse = false;
        this.servo.setPosition(0);
        this.position = -1;
    }

    // Allow construction of a servo starting at a specific position
    public CachedServo(HardwareMap hwmp, String name, double position) {
        this.servo = hwmp.get(Servo.class, name);
        this.position = position;
        this.inverse = false;
        this.servo.setPosition(this.position);
    }

    // Only set the position if it is not already at that position
    public void setPosition(double position) {
        if (this.position != position) {
            this.position = position;
            if (this.inverse) servo.setPosition(1 - this.position);
            else servo.setPosition(this.position);
            timeSinceUpdate.resetTimer();
        }
    }

    // Return cached position; servo does NOT provide encoder
    public double getPosition() {
        return this.position;
    }

    // Used for timing, easier to abstract if built in at hardware integration level
    public double secondsSinceMovement() {
        return timeSinceUpdate.getElapsedTimeSeconds();
    }

    public void resetTimer() {
        timeSinceUpdate.resetTimer();
    }

    public void inverse() {
        this.inverse = true;
    }
}