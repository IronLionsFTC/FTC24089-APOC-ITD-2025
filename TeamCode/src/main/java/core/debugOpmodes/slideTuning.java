package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.hardware.CachedMotor;
import core.parameters.HardwareParameters;

@TeleOp(name = "<--- SLIDE TUNING OPMODE --->")
public class slideTuning extends LinearOpMode {

    private CachedMotor leftMotor;
    private CachedMotor rightMotor;

    private PIDController outtakePID;

    @Config
    public static class SlideParameters {
        public static boolean usePID = false;
        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;

        public static double T = 0.0;

        public static double leftPower = 0.0;
        public static double rightPower = 0.0;
    }

    @Override
    public void runOpMode() {
        if (isStopRequested()) return;
        waitForStart();

        outtakePID = new PIDController(
            SlideParameters.P,
            SlideParameters.I,
            SlideParameters.D
        );

        leftMotor = new CachedMotor(hardwareMap, HardwareParameters.Motors.HardwareMapNames.leftOuttakeSlide);
        rightMotor = new CachedMotor(hardwareMap, HardwareParameters.Motors.HardwareMapNames.rightOuttakeSlide);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            outtakePID.setPID(
                SlideParameters.P,
                SlideParameters.I,
                SlideParameters.D
            );

            if (!SlideParameters.usePID) {
                leftMotor.setPower(SlideParameters.leftPower);
                rightMotor.setPower(SlideParameters.rightPower);
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

            telemetry.addData("leftPos", leftMotor.getPosition());
            telemetry.addData("rightPos", rightMotor.getPosition());
            telemetry.update();
        }
    }
}
