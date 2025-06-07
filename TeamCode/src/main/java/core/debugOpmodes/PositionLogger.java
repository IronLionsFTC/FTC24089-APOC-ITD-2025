package core.debugOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.computerVision.Limelight;
import core.hardware.CachedMotor;
import core.parameters.HardwareParameters;
import core.subsystems.Intake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp ( name = "Position Logger")
public class PositionLogger extends LinearOpMode {

    Follower follower;

    @Override
    public void runOpMode() {

        if (isStopRequested()) return;
        waitForStart();

        CachedMotor motor = new CachedMotor(hardwareMap, HardwareParameters.Motors.HardwareMapNames.intakeSlide);
        motor.resetEncoder();
        motor.setReversed(HardwareParameters.Motors.Reversed.intakeSlide);

        Constants.setConstants(FConstants.class, LConstants.class);
        this.follower = new Follower(hardwareMap);

        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("r", Math.toDegrees(follower.getPose().getHeading()));

            telemetry.addData("s", motor.getPosition());

            telemetry.update();
        }
    }
}
