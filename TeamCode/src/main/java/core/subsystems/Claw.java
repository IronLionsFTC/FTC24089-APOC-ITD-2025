package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.hardware.CachedServo;
import core.parameters.PositionalBounds;
import core.state.Subsystems;

public class Claw extends SubsystemBase {
    private CachedServo servo;
    private Subsystems.ClawState state;

    public Claw(HardwareMap hwmp, String name, Subsystems.ClawState startState) {
        this.servo = new CachedServo(hwmp, name);
        this.setState(startState);
    }

    public void setState(Subsystems.ClawState state) {
        this.state = state;

        switch (this.state) {
            case StrongGripClosed:
                this.servo.setPosition(PositionalBounds.ServoPositions.ClawPositions.strongGripPosition);
                break;

            case WeakGripClosed:
                this.servo.setPosition(PositionalBounds.ServoPositions.ClawPositions.weakGripPosition);
                break;

            case Open:
                this.servo.setPosition(PositionalBounds.ServoPositions.ClawPositions.openPosition);
                break;

            case WideOpen:
                this.servo.setPosition(PositionalBounds.ServoPositions.ClawPositions.wideOpenPosition);
                break;
        }
    }
}
