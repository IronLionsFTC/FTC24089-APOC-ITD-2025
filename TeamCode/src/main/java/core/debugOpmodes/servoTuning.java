package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.exceptions.UserOpModeRunningException;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import core.hardware.IndicatorLight;
import core.parameters.HardwareParameters;
import core.parameters.PositionalBounds;

@TeleOp(name = "<--- SERVO TUNING OPMODE --->")
public class servoTuning extends LinearOpMode {

    private Servo intakeYawServo;
    private Servo intakePitchServo;
    private Servo intakeClawServo;

    private Servo leftOuttakePitchServo;
    private Servo rightOuttakePitchServo;
    private Servo outtakeClawServo;
    private Servo outtakeGimble;

    private Servo flagServo;
    private Servo latchServo;

    private Servo leftIntakeServo;
    private Servo rightIntakeServo;

    private Servo lls;

    private RevColorSensorV3 outtakeProximity;

    private IndicatorLight indicator;



    @Config
    public static class ServoPositions {
        public static double intakeYaw = 0.5;
        public static double intakePitch = 0;
        public static double intakeClaw = 0.1;
        public static double outtakePitch = 0;
        public static double outtakeClaw = 0.1;
        public static double flag = 0;
        public static double latch = 0;
        public static double intakeServo = 0;
        public static double outtakeGimble = 0.3;
        public static double limelightArmPos = PositionalBounds.ServoPositions.limelightUp;

        public static double colour = 0;

        public static boolean enableLight = false;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        outtakeProximity = hardwareMap.get(RevColorSensorV3.class, HardwareParameters.Sensors.HardwareMapNames.outtakeProximity);
        indicator = new IndicatorLight(hardwareMap, "light");

        intakeYawServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.intakeYawServo);
        intakePitchServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.intakeLiftServo);
        intakeClawServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.intakeClawServo);

        outtakeGimble = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.outtakePitchServo);
        lls = hardwareMap.get(Servo.class, "limelightServo");

        leftOuttakePitchServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.leftArmServo);
        rightOuttakePitchServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.rightArmServo);
        outtakeClawServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.outtakeClawServo);

        latchServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.latchServo);

        leftIntakeServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.leftIntakeServo);
        rightIntakeServo = hardwareMap.get(Servo.class, HardwareParameters.Motors.HardwareMapNames.rightIntakeServo);

        if (isStopRequested()) { return; }
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {

            telemetry.addData("distanceThreshold", outtakeProximity.getDistance(DistanceUnit.MM));

            lls.setPosition(ServoPositions.limelightArmPos);
            intakeYawServo.setPosition(ServoPositions.intakeYaw);
            intakePitchServo.setPosition(ServoPositions.intakePitch);
            intakeClawServo.setPosition(ServoPositions.intakeClaw);
            leftOuttakePitchServo.setPosition(ServoPositions.outtakePitch);
            rightOuttakePitchServo.setPosition(1 - ServoPositions.outtakePitch);
            outtakeClawServo.setPosition(ServoPositions.outtakeClaw);
            //flagServo.setPosition(ServoPositions.flag);
            latchServo.setPosition(ServoPositions.latch);

            leftIntakeServo.setPosition(ServoPositions.intakeServo);
            rightIntakeServo.setPosition(1 - ServoPositions.intakeServo);
            outtakeGimble.setPosition(ServoPositions.outtakeGimble);

            indicator.setColour(ServoPositions.colour);
            indicator.setPower(ServoPositions.enableLight);

            telemetry.update();
        }
    }
}
