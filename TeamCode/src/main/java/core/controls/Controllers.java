package core.controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controllers {

    // Gamepad interface
    private GamepadEx gamepad1;
    private GamepadEx gamepad2;

    public Controllers(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);
    }
}
