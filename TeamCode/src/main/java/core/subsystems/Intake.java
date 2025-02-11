package core.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.hardware.MasterSlaveMotorPair;
import core.parameters.HardwareParameters;
import core.parameters.pidfCoefficients;

public class Intake extends SubsystemBase {

    // Subsystems of intake
    private LinearSlides slides;
    private Claw claw;

    // Hardware Interface / Controllers
    private PIDController slideController;
    private MasterSlaveMotorPair slideMotors;

    public Intake(HardwareMap hwmp) {
        this.slideMotors = new MasterSlaveMotorPair(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeSlide, HardwareParameters.Motors.Reversed.intakeSlide);
        this.slideController = new PIDController(
                pidfCoefficients.IntakeSlides.p,
                pidfCoefficients.IntakeSlides.i,
                pidfCoefficients.IntakeSlides.d
        );
        this.slides = new LinearSlides(this.slideMotors, this.slideController, pidfCoefficients.IntakeSlides.f, 150);
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeClawServo);
        CommandScheduler.getInstance().registerSubsystem(this.slides);
    }

    public void nextState() {
        // Internal intake state machine - robot specific
        // Logical flow is as follows:
        // Retracted -> Extended (claw up) -> Extended (claw down) -> Extended (grabbing) -> Transfer -> Retracted (back to start)

    }
}
