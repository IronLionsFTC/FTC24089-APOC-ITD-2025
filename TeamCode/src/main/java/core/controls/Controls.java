package core.controls;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.DoubleSupplier;

public class Controls {
    public static class Buttons {

        // GamepadEx interfaces
        public GamepadEx gamepad1;
        public GamepadEx gamepad2;

        // Buttons that trigger specific commands
        public GamepadButton intakeCycle;
        public GamepadButton outtakeCycle;
        public GamepadButton rotateLeft;
        public GamepadButton rotateRight;
        public GamepadButton override;

        // Controls to be constantly read
        public DoubleSupplier rotateClawLeft;
        public DoubleSupplier rotateClawRight;
        public DoubleSupplier driveY;
        public DoubleSupplier driveX;
        public DoubleSupplier yaw;

        public Buttons(Gamepad gamepad1, Gamepad gamepad2) {

            this.gamepad1 = new GamepadEx(gamepad1);
            this.gamepad2 = new GamepadEx(gamepad2);

            // Buttons
            this.intakeCycle = this.gamepad1.getGamepadButton(GamepadKeys.Button.X);
            this.outtakeCycle = this.gamepad1.getGamepadButton(GamepadKeys.Button.A);
            this.rotateLeft = this.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
            this.rotateRight = this.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            this.override = this.gamepad1.getGamepadButton(GamepadKeys.Button.Y);

            // Lambda to pass arguments as a double supplier
            this.rotateClawLeft = () -> this.gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            this.rotateClawRight = () -> this.gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

            // Method reference to avoid creating a lambda
            this.driveY = this.gamepad1::getLeftY;
            this.driveX = this.gamepad1::getLeftX;
            this.yaw = this.gamepad1::getRightX;
        }
    }
}
