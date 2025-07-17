package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import core.hardware.CachedMotor;
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
    private CachedMotor leftSlideMotor;
    private CachedMotor rightSlideMotor;
    private CachedServo pitchServo;

    // Arm handler - this COULD be a subsystem, for now it is not.
    private Arm arm;
    public boolean transferComplete;

    // Telemetry
    private Telemetry telemetry;

    // PID
    private PIDController leftSlideController;
    private PIDController rightSlideController;

    // State
    public OuttakeState state;
    private boolean useHighBasket;
    private boolean hasCycleOccured = false;
    public boolean pitchUp;
    public boolean wasSpec;
    public boolean lower;
    private DoubleSupplier delta;
    public boolean hasHung;
    public boolean hasWinched;

    private CachedServo hangServo;
    private CachedMotor cachedMotor;
    private boolean hasGoneDown;

    private boolean hasFlipped;

    public Outtake(HardwareMap hwmp, Telemetry telemetry, BooleanSupplier forceDown) {
        this.useHighBasket = true;
        this.arm = new Arm(hwmp);
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.outtakeClawServo);
        this.pitchServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.outtakePitchServo);
        this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleTransfer);
        this.transferComplete = true;
        this.hangServo = new CachedServo(hwmp, "hangServo");
        this.hangServo.setPosition(0.5);
        this.cachedMotor = new CachedMotor(hwmp, "hangMotor");
        this.cachedMotor.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.FLOAT);
        this.hasHung = false;
        this.hasWinched = false;
        this.hasFlipped = false;
        this.hasGoneDown = false;

        // Currently start with claw closed for preloads, always use high basket
        this.state = OuttakeState.DownClawClosed;
        this.useHighBasket = true;

        this.telemetry = telemetry;
        this.leftSlideMotor = new CachedMotor(hwmp, HardwareParameters.Motors.HardwareMapNames.leftOuttakeSlide);
        this.leftSlideMotor.setReversed(HardwareParameters.Motors.Reversed.leftOuttakeSlide);
        this.rightSlideMotor = new CachedMotor(hwmp, HardwareParameters.Motors.HardwareMapNames.rightOuttakeSlide);
        this.rightSlideMotor.setReversed(HardwareParameters.Motors.Reversed.rightOuttakeSlide);
        this.leftSlideMotor.resetEncoder();
        this.rightSlideMotor.resetEncoder();

        this.leftSlideController = new PIDController(
                pidfCoefficients.OuttakeSlides.p,
                pidfCoefficients.OuttakeSlides.i,
                pidfCoefficients.OuttakeSlides.d
        );

        this.rightSlideController = new PIDController(
                pidfCoefficients.OuttakeSlides.p,
                pidfCoefficients.OuttakeSlides.i,
                pidfCoefficients.OuttakeSlides.d
        );

        VoltageSensor voltageSensor = hwmp.voltageSensor.get("Control Hub");

        this.slides = new LinearSlides(this.leftSlideMotor, this.rightSlideMotor, this.leftSlideController, this.rightSlideController, telemetry, voltageSensor,
                pidfCoefficients.OuttakeSlides.feedforward, pidfCoefficients.OuttakeSlides.feedbackward,
                PositionalBounds.SlidePositions.outtakeMaximumExtension, forceDown, () -> false);
        this.slides.setZeroPowerOnRetraction();

        this.claw.setState(Subsystems.ClawState.WeakGripClosed);
        this.pitchUp = false;
        this.wasSpec = false;
        this.lower = true;
        this.hasFlipped = false;
        this.delta = () -> 0;
    }

    public Outtake(HardwareMap hwmp, Telemetry telemetry, BooleanSupplier forceDown, BooleanSupplier zeroing, DoubleSupplier delta) {
        this.delta = delta;
        this.useHighBasket = true;
        this.arm = new Arm(hwmp);
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.outtakeClawServo);
        this.pitchServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.outtakePitchServo);
        this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleTransfer);
        this.cachedMotor = new CachedMotor(hwmp, "hangMotor");
        this.cachedMotor.setZeroPowerBehaviour(Motor.ZeroPowerBehavior.FLOAT);
        this.transferComplete = true;
        this.hangServo = new CachedServo(hwmp, "hangServo");
        this.hangServo.setPosition(0.5);
        this.lower = false;
        this.hasHung = false;
        this.hasWinched = false;
        this.hasGoneDown = false;

        // Currently start with claw closed for preloads, always use high basket
        this.state = OuttakeState.DownClawClosed;
        this.useHighBasket = true;

        this.telemetry = telemetry;
        this.leftSlideMotor = new CachedMotor(hwmp, HardwareParameters.Motors.HardwareMapNames.leftOuttakeSlide);
        this.leftSlideMotor.setReversed(HardwareParameters.Motors.Reversed.leftOuttakeSlide);
        this.rightSlideMotor = new CachedMotor(hwmp, HardwareParameters.Motors.HardwareMapNames.rightOuttakeSlide);
        this.rightSlideMotor.setReversed(HardwareParameters.Motors.Reversed.rightOuttakeSlide);
        this.leftSlideMotor.resetEncoder();
        this.rightSlideMotor.resetEncoder();

        this.leftSlideController = new PIDController(
                pidfCoefficients.OuttakeSlides.p,
                pidfCoefficients.OuttakeSlides.i,
                pidfCoefficients.OuttakeSlides.d
        );

        this.rightSlideController = new PIDController(
                pidfCoefficients.OuttakeSlides.p,
                pidfCoefficients.OuttakeSlides.i,
                pidfCoefficients.OuttakeSlides.d
        );

        VoltageSensor voltageSensor = hwmp.voltageSensor.get("Control Hub");

        this.slides = new LinearSlides(this.leftSlideMotor, this.rightSlideMotor, this.leftSlideController, this.rightSlideController, telemetry, voltageSensor,
                pidfCoefficients.OuttakeSlides.feedforward, pidfCoefficients.OuttakeSlides.feedbackward,
                PositionalBounds.SlidePositions.outtakeMaximumExtension, forceDown, zeroing);
        this.slides.setZeroPowerOnRetraction();

        this.claw.setState(Subsystems.ClawState.WeakGripClosed);
        this.pitchUp = false;
        this.wasSpec = false;
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
            this.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown);
        }

        public void setArmPosition(double position) {
            this.leftArmServo.setPosition(position);
            this.rightArmServo.setPosition(1 - position);
        }

        public double getArmPosition() {
            return this.leftArmServo.getPosition();
        }

        public boolean armPhysicallyUp() { return this.leftArmServo.getPosition() >= 0.6 && this.leftArmServo.secondsSinceMovement() > Timings.armFoldUpTime; }
        public boolean armPhysicallyDown() { return this.leftArmServo.getPosition() <= 0.6 && this.leftArmServo.secondsSinceMovement() > Timings.armFoldDownTime; }
    }

    // Wrapper function for getting height based on basket substate
    public double getTargetHeight() {
        if (this.useHighBasket){
            return PositionalBounds.SlidePositions.OuttakePositions.highBasket;
        } else {
            return PositionalBounds.SlidePositions.OuttakePositions.lowBasket;
        }
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
            this.leftSlideController.setPID(
                    pidfCoefficients.OuttakeSlides.p,
                    pidfCoefficients.OuttakeSlides.i,
                    pidfCoefficients.OuttakeSlides.d
            );

            this.rightSlideController.setPID(
                    pidfCoefficients.OuttakeSlides.p,
                    pidfCoefficients.OuttakeSlides.i,
                    pidfCoefficients.OuttakeSlides.d
            );

            this.slides.setFeedForward(pidfCoefficients.OuttakeSlides.feedforward);
            this.slides.setFeedBackward(pidfCoefficients.OuttakeSlides.feedbackward);
        }
        this.telemetry.addData("ext", this.slides.getPosition());

        double offset = delta.getAsDouble() / 10;

        // Internal state machine
        switch (this.state) {
            case DownClawOpen:


                this.hasFlipped = false;

                if (this.areSlidesDown() && this.hasHung) {
                    this.hangServo.setPosition(0.7);
                }

                this.transferComplete = false;
                if (((this.arm.armPhysicallyDown() || !hasCycleOccured) && this.clawOpened()) || this.slides.getRelative() < 0.2) this.slides.setTarget(0);
                else {
                    if (this.useHighBasket) this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.highBasket);
                    else this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.lowBasket);
                }

                if (this.wasSpec) this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake * 1.5);

                //if (!this.arm.armPhysicallyDown()) this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleTransfer);
                //else this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleTransfer);
                this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleTransfer);
                if (this.slides.atTarget() && this.wasSpec) this.wasSpec = false;

                this.claw.setState(Subsystems.ClawState.Open);
                if (this.pitchUp) this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown + 0.1);
                else this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown);

                if (this.hasWinched) {
                    this.slides.disable();
                    if (this.slides.getRelative() > 0.8) {
                        this.cachedMotor.setPower(0.8);
                    } else {
                        this.cachedMotor.setPower(0.5);
                    }
                }

                if (this.atTargetHeight() && !this.slides.isForcedDown()) {
                    this.slides.disable();
                } else {
                    this.slides.enable();
                }

                break;

            case DownClawClosed:
                this.slides.enable();
                this.hasFlipped = false;
                if (this.wasSpec) this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake * 1.5);
                else this.slides.setTarget(0.0);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);

                if (this.slides.atTarget() && this.wasSpec) this.wasSpec = false;

                if (!this.transferComplete) {
                    this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown - 0.05);
                    this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleTransfer);
                } else {
                    this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSample);
                    this.pitchServo.setPosition(0.28);
                }
                break;

            case UpClawClosed:
                this.slides.enable();
                if (this.hasHung) {
                    if (!this.slides.atTarget() && this.getTargetHeight() > 0.4) {
                        this.cachedMotor.setPower(-1);
                    } else {
                        this.cachedMotor.setPower(0);
                        this.hasWinched = true;
                    }
                    offset += 0.2;
                }

                this.slides.setTarget(this.getTargetHeight() + offset);

                if (this.hasHung) this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown + 0.1);
                else this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSample);

                double deltaOffset = 0;
                if (this.delta.getAsDouble() > 0) {
                    deltaOffset = 0.05;
                }
                if (!this.slides.nearlyAtTarget() && !this.hasFlipped) this.pitchServo.setPosition(0.28);
                else {
                    this.hasFlipped = true;
                    if (this.lower) this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleOuttake + deltaOffset);
                    else this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleOuttake - 0.1 + deltaOffset);
                }
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.hasCycleOccured = true;
                break;

            case UpClawOpen:
                this.slides.enable();
                this.slides.setTarget(this.getTargetHeight() + offset);
                this.claw.setState(Subsystems.ClawState.Open);
                if (this.hasHung) this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown + 0.1);
                else this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSample);
                double adeltaOffset = 0;
                if (this.delta.getAsDouble() > 0) {
                    adeltaOffset = 0.05;
                }
                if (this.lower) this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleOuttake + adeltaOffset);
                else this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSampleOuttake - 0.1 + adeltaOffset);

                // Automatically retract outtake when the sample has been dropped
                if (this.claw.hasClawPhysicallyOpened()) this.nextState();
                this.hasCycleOccured = true;
                break;

            case SpecimenIntakeClawOpen:
                this.slides.enable();
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSpecimen);

                if (this.armPhysicallyDown()) this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake * 1.0);
                else this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake * 1.5);
                this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSpecimenIntake);

                if (this.pitchServo.elapsedTime() > 1.0) {
                    this.claw.setState(Subsystems.ClawState.Open);
                } else {
                    this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                }
                break;

            case SpecimenIntakeClawClosed:
                this.slides.enable();
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);

                if (this.clawClosed()) this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake * 1.5);
                else this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake * 1.0);

                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armSpecimen);
                this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSpecimenIntake);
                this.hasCycleOccured = false;
                if (this.clawClosed() && this.slides.atTarget()) nextState();
                break;

            case SpecimenOuttakeEntry:
                this.slides.enable();
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown);
                this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSpecimenEntry);
                this.hasCycleOccured = false;

                if (this.pitchServo.elapsedTime() > 0.4) {
                    this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake);
                    this.claw.setState(Subsystems.ClawState.ReallyWeak);
                }
                else {
                    this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake * 1.4);
                    this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                }

                break;

            case SpecimenOuttakeExit:
                this.slides.enable();
                this.slides.setTarget(PositionalBounds.SlidePositions.OuttakePositions.specimenOuttake * 3);
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                this.arm.setArmPosition(PositionalBounds.ServoPositions.Outtake.armDown);
                this.pitchServo.setPosition(PositionalBounds.ServoPositions.Outtake.pitchSpecimenEntry);
                this.hasCycleOccured = false;
                this.wasSpec = false;
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
        return (this.state == OuttakeState.DownClawClosed || this.state == OuttakeState.DownClawOpen) && this.slides.getRelative() < 0.2;
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
