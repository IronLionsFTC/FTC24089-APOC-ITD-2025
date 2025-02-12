package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import core.parameters.HardwareParameters;

@TeleOp(name = "<--- SERVO TUNING OPMODE --->")
public class servoTuning extends LinearOpMode {

    private Servo intakeYawServo;
    private Servo intakePitchServo;
    private Servo intakeClawServo;

    private Servo leftOuttakePitchServo;
    private Servo rightOuttakePitchServo;
    private Servo outtakeClawServo;

    private Servo flagServo;
    private Servo latchServo;

    @Config
    public static class ServoPositions {
        public static double intakeYaw = 0;
        public static double intakePitch = 0;
        public static double intakeClaw = 0;
        public static double outtakePitch = 0;
        public static double outtakeClaw = 0.7;
        public static double flag = 0;
        public static double latch = 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        intakeYawServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.intakeYawServo);
        intakePitchServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.intakeLiftServo);
        intakeClawServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.intakeClawServo);

        leftOuttakePitchServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.leftArmServo);
        rightOuttakePitchServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.rightArmServo);
        outtakeClawServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.outtakeClawServo);

        latchServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.latchServo);

        if (isStopRequested()) { return; }
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {

            intakeYawServo.setPosition(ServoPositions.intakeYaw);
            intakePitchServo.setPosition(ServoPositions.intakePitch);
            intakeClawServo.setPosition(ServoPositions.intakeClaw);
            leftOuttakePitchServo.setPosition(ServoPositions.outtakePitch);
            rightOuttakePitchServo.setPosition(1 - ServoPositions.outtakePitch);
            outtakeClawServo.setPosition(ServoPositions.outtakeClaw);
            //flagServo.setPosition(ServoPositions.flag);
            latchServo.setPosition(ServoPositions.latch);
        }
    }
}
