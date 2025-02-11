package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.hardware.MasterSlaveMotorPair;
import core.parameters.HardwareParameters;

public class Intake extends SubsystemBase {
    private LinearSlides slides;
    private PIDController slideController;
    private MasterSlaveMotorPair slideMotors;

    public Intake(HardwareMap hwmp) {
        this.slideMotors = new MasterSlaveMotorPair(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeSlide, HardwareParameters.Motors.Reversed.intakeSlide);
        this.slideController = new LinearSlides()
    }
}
