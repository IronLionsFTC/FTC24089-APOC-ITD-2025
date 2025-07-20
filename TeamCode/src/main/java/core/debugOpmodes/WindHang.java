package core.debugOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import core.hardware.CachedMotor;

@Autonomous
public class WindHang extends LinearOpMode {
    public void runOpMode() {

        CachedMotor motor = new CachedMotor(hardwareMap, "hangMotor");

        if (isStopRequested()) return;
        waitForStart();

        while (this.opModeIsActive()) {
            motor.setPower(1);
        }

        motor.setPower(0);
    }
}
