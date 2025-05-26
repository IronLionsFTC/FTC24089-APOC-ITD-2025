package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.math.Utility;
import core.parameters.PositionalBounds.ServoPositions.ClawPositions;
import core.hardware.CachedServo;
import core.parameters.Timings;

public class DualAxisGimbal extends SubsystemBase {

    // Servo / hardware interface
    // assume pitch and yaw, such that the yaw is ON the pitch
    private CachedServo leftPitchServo;
    private CachedServo rightPitchServo;
    // Assume 0.5 is the resting position for the yaw servo so that it can rotate in either direction

    private CachedServo yawServo;

    private double tilt = 0;

    public DualAxisGimbal(HardwareMap hwmp, String leftPitchServoName, String rightPitchServoName, String yawServoName) {
        this.leftPitchServo = new CachedServo(hwmp, leftPitchServoName);
        this.rightPitchServo = new CachedServo(hwmp, rightPitchServoName);
        this.yawServo = new CachedServo(hwmp, yawServoName, ClawPositions.yawRest);
        this.yawServo.setPosition(ClawPositions.yawRest);
        this.leftPitchServo.setPosition(0);
    }

    public void resetPosition() {
        this.yawServo.setPosition(ClawPositions.yawRest);

        this.leftPitchServo.setPosition(ClawPositions.pitchRest);
        this.rightPitchServo.setPosition(1 - ClawPositions.pitchRest);
    }

    public void extendPitch() {
        this.leftPitchServo.setPosition(ClawPositions.pitchExtended - this.tilt);
        this.rightPitchServo.setPosition(1 - ClawPositions.pitchExtended - this.tilt);
    }

    public void rotateYaw(double speed) {
        this.yawServo.setPosition(Utility.clamp(yawServo.getPosition() + (speed / -5),
                ClawPositions.yawRest - ClawPositions.yawRange, ClawPositions.yawRest + ClawPositions.yawRange));
    }

    public void setYaw(double yaw) {
        this.yawServo.setPosition(yaw);
    }

    public boolean foldedUp() {
        return this.leftPitchServo.secondsSinceMovement() > Timings.clawFoldUpTime && this.leftPitchServo.getPosition() == ClawPositions.pitchRest;
    }

    public boolean foldedDown() {
        return this.leftPitchServo.secondsSinceMovement() > Timings.clawFoldDownTime && this.leftPitchServo.getPosition() == ClawPositions.pitchExtended;
    }

    public boolean doneFolding() {
        if (this.foldedDown()) {
            return true;
        } else return this.foldedUp();
    }

    public double getYaw() {
        return this.yawServo.getPosition();
    }

    public void setTilt(double newTilt) {
        this.tilt = newTilt;
    }
}
