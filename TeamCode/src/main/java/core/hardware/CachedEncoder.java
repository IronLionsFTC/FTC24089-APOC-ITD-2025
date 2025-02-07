package core.hardware;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CachedEncoder {
    private MotorEx encoderHandle;
    private double value = 0;

    public CachedEncoder(HardwareMap hwmp, String name) {
        this.encoderHandle = new MotorEx(hwmp, name);
        this.encoderHandle.resetEncoder();
    }

    public void reset() {
        this.encoderHandle.resetEncoder();
        this.value = 0;
    }

    public double read() {
        this.value = encoderHandle.getCurrentPosition();
        return value;
    }

    public double lastRead() {
        return this.value;
    }
}
