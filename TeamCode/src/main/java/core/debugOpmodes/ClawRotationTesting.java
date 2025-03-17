package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.computerVision.Limelight;
import core.hardware.CachedServo;
import core.parameters.HardwareParameters;

@TeleOp ( name = "COMPUTER VISION TEST")
public class ClawRotationTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Limelight limelight = new Limelight(hardwareMap);
        CachedServo clawYaw = new CachedServo(hardwareMap, HardwareParameters.Motors.HardwareMapNames.intakeYawServo);

        if (isStopRequested()) return;
        waitForStart();

        limelight.enable();

        while (opModeIsActive()) {
            telemetry.update();

            Limelight.SampleState sampleState = limelight.query();
            telemetry.addData("IS NULL", sampleState == null);
            if (sampleState == null) continue;

            telemetry.addData("ANGLE", sampleState.angle);

            double angle = sampleState.angle / 355;
            clawYaw.setPosition(0.5 + angle);
        }
    }
}
