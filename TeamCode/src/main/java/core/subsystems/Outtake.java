package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.hardware.CachedServo;
import core.hardware.MasterSlaveMotorPair;
import core.parameters.HardwareParameters;
import core.parameters.PositionalBounds;
import core.parameters.pidfCoefficients;
import core.state.Subsystems;
import core.state.Subsystems.OuttakeState;

public class Outtake extends SubsystemBase {

    // Subsystems
    private LinearSlides slides;
    private Claw claw;
    private MasterSlaveMotorPair slideMotors;

    // Servos - note that one of these must be the inverse (1 - x) of the other
    // As of current configuration, right is the inversed one.
    private CachedServo leftArmServo;
    private CachedServo rightArmServo;

    // Telemetry
    private Telemetry telemetry;

    // PID
    private PIDController slideController;

    // State
    private OuttakeState state;

    public Outtake(HardwareMap hwmp, Telemetry telemetry) {

        // Load the servos directly
        this.leftArmServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.leftArmServo);
        this.rightArmServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.rightArmServo);

        // Currently start with claw open
        this.state = OuttakeState.DownClawOpen;
        this.telemetry = telemetry;
        this.slideMotors = new MasterSlaveMotorPair(hwmp,
                HardwareParameters.Motors.HardwareMapNames.rightOuttakeSlide,
                HardwareParameters.Motors.Reversed.rightOuttakeSlide,
                HardwareParameters.Motors.HardwareMapNames.leftOuttakeSlide,
                HardwareParameters.Motors.Reversed.leftOuttakeSlide);

        this.slideController = new PIDController(
                pidfCoefficients.OuttakeSlides.p,
                pidfCoefficients.OuttakeSlides.i,
                pidfCoefficients.OuttakeSlides.d
        );

        this.slides = new LinearSlides(this.slideMotors, this.slideController, telemetry,
                pidfCoefficients.OuttakeSlides.feedforward, pidfCoefficients.OuttakeSlides.feedbackward,
                PositionalBounds.SlidePositions.outtakeMaximumExtension);
    }

    // Wrapper around setting the positions such that the right servo is inverted,
    // prevents me having to set both every time potentially forgetting an inversion
    private void setArmPosition(double position) {
        this.leftArmServo.setPosition(position);
        this.rightArmServo.setPosition(1 - position);
    }

    @Override
    public void periodic() {

        // Set the PIDF coefficients constantly for tuning purposes
        if (pidfCoefficients.OuttakeSlides.tuning) {
            this.slideController.setPID(
                    pidfCoefficients.OuttakeSlides.p,
                    pidfCoefficients.OuttakeSlides.i,
                    pidfCoefficients.OuttakeSlides.d
            );

            this.slides.setFeedForward(pidfCoefficients.OuttakeSlides.feedforward);
            this.slides.setFeedBackward(pidfCoefficients.OuttakeSlides.feedbackward);
        }

        // Internal state machine
        switch (this.state) {
            case DownClawOpen:
                this.slides.setTarget(0);
                this.claw.setState(Subsystems.ClawState.Open);
                this.setArmPosition(0);
                break;
            case DownClawClosed:
                this.slides.setTarget(0);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.setArmPosition(0);
                break;

            case HighBasketUpClawClosed:
                this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.highBasket);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.setArmPosition(0.7);
                break;
            case HighBasketUpClawOpen:
                this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.highBasket);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.setArmPosition(0.7);
                break;

            case LowBasketUpClawClosed:
                this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.highBasket);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.setArmPosition(0.7);
                break;

        }
    }
}
