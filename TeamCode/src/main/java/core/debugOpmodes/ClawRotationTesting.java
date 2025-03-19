package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.hardware.CachedServo;
import core.parameters.HardwareParameters;

@TeleOp ( name = "COMPUTER VISION TEST")
public class ClawRotationTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(60);
        CachedServo clawYaw = new CachedServo(hardwareMap, HardwareParameters.Motors.HardwareMapNames.intakeYawServo);

        if (isStopRequested()) return;
        waitForStart();

        limelight.pipelineSwitch(0);
        limelight.start();

        while (opModeIsActive()) {
            sleep(50);

            limelight.updatePythonInputs(0, 0, 0, 0, 0, 0, 0, 0);

            telemetry.addData("RUNNING", limelight.isRunning());
            telemetry.addData("CONNECTED", limelight.isConnected());
            telemetry.update();

            if (gamepad1.a) {
                telemetry.addLine("STARTING");
                limelight.start();
            }

            if (gamepad1.dpad_up) {
                telemetry.addLine("STOPPING");
                limelight.stop();
            }

            if (gamepad1.dpad_left) {
                telemetry.addLine("SHUTTING DOWN");
                limelight.shutdown();
            }

            if (gamepad1.b) limelight.pipelineSwitch(0);
            if (gamepad1.x) limelight.pipelineSwitch(2);
            if (gamepad1.y) {
                limelight.resetDeviceConfigurationForOpMode();
                limelight.reloadPipeline();
            }

            telemetry.addLine(limelight.getStatus().toString());


            LLResult result = limelight.getLatestResult();
            telemetry.addData("Result NULL", result == null);
            if (result == null) continue;

            telemetry.addData("X", result.getTx());

            double[] pythonResult = result.getPythonOutput();
            telemetry.addData("Python NULL", pythonResult == null);
            if (pythonResult == null) continue;
            telemetry.addData("ANGLE", pythonResult[0]);
            double angle = pythonResult[0];

            clawYaw.setPosition(0.5 - angle / 355);
        }
    }
}
