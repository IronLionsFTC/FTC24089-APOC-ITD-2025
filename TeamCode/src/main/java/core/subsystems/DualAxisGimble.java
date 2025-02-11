package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.math.Utility;
import core.parameters.PositionalBounds.ServoPositions.ClawPositions;
import core.hardware.CachedServo;

public class DualAxisGimble extends SubsystemBase {

    // Servo / hardware interface
    // assume pitch and yaw, such that the yaw is ON the pitch
    private CachedServo pitchServo;
    // Assume 0.5 is the resting position for the yaw servo so that it can rotate in either direction
    private CachedServo yawServo;

    public DualAxisGimble(HardwareMap hwmp, String pitchServoName, String yawServoName) {
        this.pitchServo = hwmp.get(CachedServo.class, pitchServoName);
        this.yawServo = hwmp.get(CachedServo.class, yawServoName);
    }

    public DualAxisGimble(CachedServo pitchServo, CachedServo yawServo) {
        this.pitchServo = pitchServo;
        this.yawServo = yawServo;
    }

    public void resetPosition() {
        this.yawServo.setPosition(ClawPositions.yawRest);
        this.pitchServo.setPosition(ClawPositions.pitchRest);
    }

    public void extendPitch() {
        this.pitchServo.setPosition(ClawPositions.pitchExtended);
    }

    public void rotateYaw(double speed) {
        this.yawServo.setPosition(Utility.clamp(yawServo.getPosition() + (speed / 50),
                ClawPositions.yawRest - ClawPositions.yawRange, ClawPositions.yawRest + ClawPositions.yawRange));
    }

    public void setYaw(double yaw) {
        this.yawServo.setPosition(ClawPositions.yawRest);
    }
}
