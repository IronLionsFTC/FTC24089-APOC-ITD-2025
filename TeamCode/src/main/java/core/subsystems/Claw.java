package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.hardware.CachedServo;
import core.parameters.PositionalBounds;
import core.parameters.Timings;
import core.state.Subsystems;

public class Claw extends SubsystemBase {
    private CachedServo servo;
    private Subsystems.ClawState state;

    // Expose a constructor for creating a claw subsystem starting open
    public Claw(HardwareMap hwmp, String name) {
        this.servo = new CachedServo(hwmp, name);
        this.setState(Subsystems.ClawState.WideOpen);
    }

    // Expose a constructor allowing the claw to be initialised in a certain state (e.g. for preloading)
    public Claw(HardwareMap hwmp, String name, Subsystems.ClawState startState) {
        this.servo = new CachedServo(hwmp, name);
        this.setState(startState);
    }

    public void setState(Subsystems.ClawState state) {
        this.state = state;

        // Decide where to set the position based on the parameters in the dedicated file
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

    public boolean hasClawPhysicallyOpened() {
        // If the state is open
        return (this.state == Subsystems.ClawState.Open || this.state == Subsystems.ClawState.WideOpen)
                // Wait a certain amount of time prior to moving on to prevent mechanism jamming
                && this.servo.secondsSinceMovement() > Timings.clawOpeningTime;
    }

    public boolean hasClawPhysicallyClosed() {
        // If the state is open
        return (this.state == Subsystems.ClawState.StrongGripClosed || this.state == Subsystems.ClawState.WeakGripClosed)
                // Wait a certain amount of time prior to moving on to prevent dropping objects
                && this.servo.secondsSinceMovement() > Timings.clawClosingTime;
    }
}
