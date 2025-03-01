package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.math.Utility;
import core.parameters.PositionalBounds.ServoPositions.ClawPositions;
import core.hardware.CachedServo;
import core.parameters.Timings;

public class DualAxisGimble extends SubsystemBase {

    // Servo / hardware interface
    // assume pitch and yaw, such that the yaw is ON the pitch
    private CachedServo pitchServo;
    // Assume 0.5 is the resting position for the yaw servo so that it can rotate in either direction
    private CachedServo yawServo;

    public DualAxisGimble(HardwareMap hwmp, String pitchServoName, String yawServoName) {
        this.pitchServo = new CachedServo(hwmp, pitchServoName);
        this.yawServo = new CachedServo(hwmp, yawServoName, ClawPositions.yawRest);
        this.yawServo.setPosition(ClawPositions.yawRest);
        this.pitchServo.setPosition(0);
    }

    public void resetPosition() {
        this.yawServo.setPosition(ClawPositions.yawRest);
        this.pitchServo.setPosition(ClawPositions.pitchRest);
    }

    public void extendPitch() {
        this.pitchServo.setPosition(ClawPositions.pitchExtended);
    }

    public void rotateYaw(double speed) {
        this.yawServo.setPosition(Utility.clamp(yawServo.getPosition() + (speed / -5),
                ClawPositions.yawRest - ClawPositions.yawRange, ClawPositions.yawRest + ClawPositions.yawRange));
    }

    public void setYaw(double yaw) {
        this.yawServo.setPosition(yaw);
    }

    public boolean foldedUp() {
        return this.pitchServo.secondsSinceMovement() > Timings.clawFoldUpTime && this.pitchServo.getPosition() == ClawPositions.pitchRest;
    }

    public boolean doneFolding() {
        if (this.pitchServo.secondsSinceMovement() > Timings.clawFoldDownTime && this.pitchServo.getPosition() == ClawPositions.pitchExtended) {
            return true;
        } else if (this.pitchServo.secondsSinceMovement() > Timings.clawFoldUpTime && this.pitchServo.getPosition() == ClawPositions.pitchRest) {
            return true;
        } else {
            return false;
        }
    }

    public double getYaw() {
        return this.yawServo.getPosition();
    }
}
