package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.hardware.CachedServo;
import core.state.Subsystems;

public class Claw extends SubsystemBase {
    private CachedServo servo;
    private Subsystems.ClawState state;

    private static final double strongGripPosition = 0;
    private static final double weakGripPosition = 0.05;
    private static final double openPosition = 0.4;
    private static final double wideOpenPosition = 0.5;

    public Claw(HardwareMap hwmp, String name, Subsystems.ClawState startState) {
        this.servo = new CachedServo(hwmp, name);
        this.setState(startState);
    }

    public void setState(Subsystems.ClawState state) {
        this.state = state;

        switch (this.state) {
            case StrongGripClosed:
                this.servo.setPosition(strongGripPosition);
                break;

            case WeakGripClosed:
                this.servo.setPosition(weakGripPosition);
                break;

            case Open:
                this.servo.setPosition(openPosition);
                break;

            case WideOpen:
                this.servo.setPosition(wideOpenPosition);
                break;
        }
    }
}
