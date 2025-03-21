package core.subsystems;

import android.icu.text.Transliterator;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
    private CachedServo pitchServo;

    // Arm handler - this COULD be a subsystem, for now it is not.
    private Arm arm;
    public boolean transferComplete;

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
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.outtakeClawServo);
        this.pitchServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.outtakePitchServo);
        this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleTransfer);
        this.transferComplete = true;

        // Currently start with claw closed for preloads, always use high basket
        this.state = OuttakeState.DownClawClosed;
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

        VoltageSensor voltageSensor = hwmp.voltageSensor.get("Control Hub");

        this.slides = new LinearSlides(this.slideMotors, this.slideController, telemetry, voltageSensor,
                pidfCoefficients.OuttakeSlides.feedforward, pidfCoefficients.OuttakeSlides.feedbackward,
                PositionalBounds.SlidePositions.outtakeMaximumExtension);
        this.slides.setZeroPowerOnRetraction();

        this.claw.setState(Subsystems.ClawState.WeakGripClosed);
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
            this.setArmPosition(0);
        }

        public void setArmPosition(double position) {
            this.leftArmServo.setPosition(position);
            this.rightArmServo.setPosition(1 - position);
        }

        public double getArmPosition() {
            return this.leftArmServo.getPosition();
        }

        public boolean armPhysicallyUp() { return this.leftArmServo.getPosition() > 0.2 && this.leftArmServo.secondsSinceMovement() > Timings.armFoldUpTime; }
        public boolean armPhysicallyDown() { return this.leftArmServo.getPosition() < 0.2 && this.leftArmServo.secondsSinceMovement() > Timings.armFoldDownTime; }
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
                break;

            case SpecimenIntakeClawOpen:
                this.state = OuttakeState.SpecimenIntakeClawClosed;
                break;
            case SpecimenIntakeClawClosed:
                this.state = OuttakeState.SpecimenOuttakeEntry;
                break;
            case SpecimenOuttakeEntry:
                this.state = OuttakeState.SpecimenOuttakeExit;
                break;
            case SpecimenOuttakeExit:
                this.state = OuttakeState.DownClawOpen;
                break;
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
                this.transferComplete = false;
                if (this.arm.armPhysicallyDown() || !hasCycleOccured) this.slides.setTarget(0);
                else {
                    if (this.useHighBasket) this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.highBasket);
                    else this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.lowBasket);
                }

                if (!this.arm.armPhysicallyDown()) this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.safeMovement);
                else this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleTransfer);

                this.claw.setState(Subsystems.ClawState.Open);
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown);
                break;

            case DownClawClosed:
                this.slides.setTarget(0);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);

                if (!this.transferComplete) {
                    this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown);
                    this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleTransfer);
                } else {
                    this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSample);
                    this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.safeMovement * 1.3);
                }
                break;

            case UpClawClosed:
                this.slides.setTarget(this.getTargetHeight());
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSample);

                if (!this.arm.armPhysicallyUp()) this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.safeMovement);
                else this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleOuttake);

                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.hasCycleOccured = true;
                break;

            case UpClawOpen:
                this.slides.setTarget(this.getTargetHeight());
                this.claw.setState(Subsystems.ClawState.WideOpen);
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSample);
                this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleOuttake);

                // Automatically retract outtake when the sample has been dropped
                if (this.claw.hasClawPhysicallyOpened()) this.nextState();
                this.hasCycleOccured = true;
                break;

            case SpecimenIntakeClawOpen:
                this.slides.setTarget(0);
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSpecimen);

                if (!this.arm.armPhysicallyUp()) {
                    this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.safeMovement);
                    this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                } else {
                    this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSpecimenIntake);
                    this.claw.setState(Subsystems.ClawState.WideOpen);
                }
                break;

            case SpecimenIntakeClawClosed:
                this.slides.setTarget(0);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSpecimen);
                this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSpecimenIntake);
                this.hasCycleOccured = false;
                break;

            case SpecimenOuttakeEntry:
                this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown);
                if (!this.arm.armPhysicallyDown()) this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.safeMovement * 2);
                else this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.specimenEntry);
                this.hasCycleOccured = false;
                break;

            case SpecimenOuttakeExit:
                this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown + 0.15);
                this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.specimenEntry);
                this.hasCycleOccured = false;
                break;
        }
    }

    public boolean clawOpened() {
        return this.claw.hasClawPhysicallyOpened();
    }
    public boolean atTargetHeight() { return this.slides.atTarget(); }
    public boolean clawClosed() {
        return this.claw.hasClawPhysicallyClosed();
    }

    public boolean areSlidesRaised() {
        return (this.state == OuttakeState.UpClawOpen || this.state == OuttakeState.UpClawClosed) && this.slides.getRelative() > this.getTargetHeight() * 0.8;
    }

    public boolean areSlidesDown() {
        return (this.state == OuttakeState.DownClawClosed || this.state == OuttakeState.DownClawOpen) && this.slides.getRelative() > 0.1;
    }

    public boolean armPhysicallyOver() {
        return this.arm.armPhysicallyUp();
    }

    public boolean armPhysicallyDown() {
        return this.arm.armPhysicallyDown();
    }

    public double clawTime() {
        return this.claw.time();
    }

    public Subsystems.ClawState clawState() {
        return this.claw.getState();
    }

    public void resetClawTime() {
        this.claw.resetTime();
    }
}
