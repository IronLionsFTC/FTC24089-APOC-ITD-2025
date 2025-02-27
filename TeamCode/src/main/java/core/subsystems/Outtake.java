package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.hardware.CachedServo;
import core.hardware.MasterSlaveMotorPair;
import core.parameters.HardwareParameters;
import core.parameters.PositionalBounds;
import core.parameters.Timings;
import core.parameters.pidfCoefficients;
import core.state.Subsystems;
import core.state.Subsystems.OuttakeState;

public class Outtake extends SubsystemBase {

    // Subsystems
    private LinearSlides slides;
    private Claw claw;
    private MasterSlaveMotorPair slideMotors;

    // Arm handler - this COULD be a subsystem, for now it is not.
    private Arm arm;

    // Telemetry
    private Telemetry telemetry;

    // PID
    private PIDController slideController;

    // State
    public OuttakeState state;
    private boolean useHighBasket;
    private boolean hasCycleOccured = false;

    public Outtake(HardwareMap hwmp, Telemetry telemetry) {

        this.useHighBasket = true;
        this.arm = new Arm(hwmp);
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.outtakeClawServo, true);
        this.claw.setScalar(0.5);

        // Currently start with claw open, always use high basket
        this.state = OuttakeState.DownClawOpen;
        this.useHighBasket = true;

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
        this.slides.setZeroPowerOnRetraction();
    }

    // Wrapper around setting the positions such that the right servo is inverted,
    // prevents me having to set both every time potentially forgetting an inversion

    private class Arm {

        // Servos - note that one of these must be the inverse (1 - x) of the other
        // As of current configuration, right is the inversed one.
        private CachedServo leftArmServo;
        private CachedServo rightArmServo;

        public Arm(HardwareMap hwmp) {

            // Load the servos directly
            this.leftArmServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.leftArmServo);
            this.rightArmServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.rightArmServo);

        }

        public void setArmPosition(double position) {
            this.leftArmServo.setPosition(1 - position);
            this.rightArmServo.setPosition(position);
        }

        public double getArmPosition() {
            return this.leftArmServo.getPosition();
        }

        public boolean armPhysicallyUp() { return this.getArmPosition() > 0.5 && this.leftArmServo.secondsSinceMovement() > Timings.armFoldUpTime; }
        public boolean armPhysicallyDown() { return this.getArmPosition() < 0.5 && this.leftArmServo.secondsSinceMovement() > Timings.armFoldDownTime; }
    }

    // Wrapper function for getting height based on basket substate
    public double getTargetHeight() {
        if (this.useHighBasket) return PositionalBounds.SlidePositions.OuttakePositions.highBasket;
        return PositionalBounds.SlidePositions.OuttakePositions.lowBasket;
    }

    public void toggleBasket() {
        this.useHighBasket = !this.useHighBasket;
    }

    public void nextState() {
        this.hasCycleOccured = true;
        switch (this.state) {
            case DownClawOpen:
                this.state = OuttakeState.DownClawClosed;
                break;
            case DownClawClosed:
                this.state = OuttakeState.UpClawClosed;
                break;
            case UpClawClosed:
                this.state = OuttakeState.UpClawOpen;
                break;
            case UpClawOpen:
                this.state = OuttakeState.DownClawOpen;
        }
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
                if (this.arm.armPhysicallyDown() || !hasCycleOccured) this.slides.setTarget(0);
                else {
                    if (this.useHighBasket) this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.highBasket);
                    this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.lowBasket);
                }
                this.claw.setState(Subsystems.ClawState.Open);
                this.arm.setArmPosition(0);
                break;
            case DownClawClosed:
                this.slides.setTarget(0);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.arm.setArmPosition(0);
                break;

            case UpClawClosed:
                this.slides.setTarget(this.getTargetHeight());
                if (this.slides.getRelative() < PositionalBounds.SlidePositions.OuttakePositions.highBasket * 0.7) {
                    this.arm.setArmPosition(0);
                } else {
                    this.arm.setArmPosition(0.7);
                }
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                break;
            case UpClawOpen:
                this.slides.setTarget(this.getTargetHeight());
                this.claw.setState(Subsystems.ClawState.WideOpen);
                this.arm.setArmPosition(0.7);

                // Automatically retract outtake when the sample has been dropped
                if (this.claw.hasClawPhysicallyOpened()) this.nextState();
                break;
        }
    }

    public boolean clawOpened() {
        return this.claw.hasClawPhysicallyOpened();
    }

    public boolean clawClosed() {
        return this.claw.hasClawPhysicallyClosed();
    }
}
