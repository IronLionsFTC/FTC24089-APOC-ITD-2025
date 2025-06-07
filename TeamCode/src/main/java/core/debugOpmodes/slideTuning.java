package core.debugOpmodes;

import android.transition.Slide;

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

    private CachedMotor intakeMotor;

    private PIDController outtakePID;
    private PIDController intakePID;

    @Config
    public static class SlideParameters {
        public static boolean usePID = false;
        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;

        public static double T = 0.0;

        public static double leftPower = 0.0;
        public static double rightPower = 0.0;

        public static double aP = 0;
        public static double bI = 0;
        public static double cD = 0;

        public static double intakeTarget = 0;
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

        intakePID = new PIDController(
                SlideParameters.aP,
                SlideParameters.bI,
                SlideParameters.cD
        );

        leftMotor = new CachedMotor(hardwareMap, HardwareParameters.Motors.HardwareMapNames.leftOuttakeSlide);
        rightMotor = new CachedMotor(hardwareMap, HardwareParameters.Motors.HardwareMapNames.rightOuttakeSlide);
        intakeMotor = new CachedMotor(hardwareMap, "intakeSlide");
        intakeMotor.setReversed(HardwareParameters.Motors.Reversed.intakeSlide);
        leftMotor.setReversed(true);
        rightMotor.setReversed(false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        intakeMotor.resetEncoder();

        while (opModeIsActive()) {
            outtakePID.setPID(
                SlideParameters.P,
                SlideParameters.I,
                SlideParameters.D
            );

            intakePID.setPID(
                    SlideParameters.aP,
                    SlideParameters.bI,
                    SlideParameters.cD
            );

            if (!SlideParameters.usePID) {
                leftMotor.setPower(SlideParameters.leftPower);
                rightMotor.setPower(SlideParameters.rightPower);
            } else {
                double power = outtakePID.calculate(leftMotor.getPosition(), SlideParameters.T);
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }

            double power = intakePID.calculate(intakeMotor.getPosition(), SlideParameters.intakeTarget);
            intakeMotor.setPower(power);

            telemetry.addData("leftPos", leftMotor.getPosition());
            telemetry.addData("rightPos", rightMotor.getPosition());
            telemetry.addData("intakePos", intakeMotor.getPosition());
            telemetry.update();
        }
    }
}
