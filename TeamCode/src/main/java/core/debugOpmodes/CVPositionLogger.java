package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import core.computerVision.Limelight;
import core.parameters.HardwareParameters;
import core.subsystems.Intake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp ( name = "CV Position Logger")
public class CVPositionLogger extends LinearOpMode {

    Limelight limelight;
    Limelight.SampleState buffer;

    Follower follower;
    Intake intakeSubsystem;

    @Config
    public static class LimelightInformation {
        public static double limelightHeight = 13.7;
        public static double limelightAngle = 65;
        public static double forwardOffset = 0.5;

        public static double forwardScalarForLateral = 0.01143;
        public static double forwardOffsetForLateral = 0.2661;

        public static double a = 24.32983;
        public static double b = 0.0310553;
        public static double c = -8.773041;
        public static double e = Math.E;
    }

    @Override
    public void runOpMode() {

        if (isStopRequested()) return;
        waitForStart();

        this.limelight = new Limelight(hardwareMap, Limelight.Targets.YellowOnly);
        this.buffer = new Limelight.SampleState();

        Constants.setConstants(FConstants.class, LConstants.class);
        this.follower = new Follower(hardwareMap);

        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, telemetry);


        this.limelight.enable();
        this.limelight.raise();

        while (opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Intake Ext", intakeSubsystem.getSlidePosition());
            this.intakeSubsystem.takeGimbleSubsystem().extendPitch();

            this.buffer = limelight.query(telemetry, follower, intakeSubsystem);

            if (this.buffer == null) {
                continue;
            }

            if (this.buffer.angle == 0) {
                continue;
            }

            double slideExtension = 319.77939 * Math.pow(Math.E, 0.0310912 * buffer.center.y) - 83.62055;
            double lateralGradient = -0.000871067 * slideExtension - 0.474881;
            double lateralIntercept = -10.45;

            double lateralCM = lateralGradient * buffer.center.x + lateralIntercept;

            telemetry.addData("Sample Angle", buffer.angle);
            telemetry.addData("Sample X_fov", buffer.center.x);
            telemetry.addData("Sample Y_fov", buffer.center.y);
            telemetry.addData("Calculated Ext", slideExtension);
        }
    }
}
