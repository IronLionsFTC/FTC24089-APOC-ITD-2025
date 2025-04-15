package core.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IndicatorLight {
    private Servo hardware;

    public IndicatorLight(HardwareMap hwmp, String name) {
        this.hardware = hwmp.get(Servo.class, name);
    }

    public void setColour(double colour) {
        this.hardware.setPosition(colour);
    }

    public void setPower(boolean power) {
        if (power) this.hardware.getController().pwmEnable();
        else this.hardware.getController().getPwmStatus();
    }
}
