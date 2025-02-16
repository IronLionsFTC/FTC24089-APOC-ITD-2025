package core.debugOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.hardware.MasterSlaveMotorPair;

@TeleOp(name = "<--- SLIDE TUNING OPMODE --->")
public class slideTuning extends LinearOpMode {

    private MasterSlaveMotorPair intakeSlides;
    private MasterSlaveMotorPair outtakeSlides;

    private PIDController intakePID;
    private PIDController outtakePID;

    @Config
    public static class SlideParameters {

    }

    @Override
    public void runOpMode() {

    }
}
