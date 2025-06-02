package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import core.hardware.CachedMotor;
import core.hardware.CachedServo;
import core.hardware.IndicatorLight;
import core.math.Colour;
import core.parameters.HardwareParameters;
import core.parameters.PositionalBounds;
import core.parameters.pidfCoefficients;
import core.state.Subsystems;
import core.state.Subsystems.IntakeState;

public class Intake extends SubsystemBase {

    // Subsystems of intake
    private Claw claw;
    private DualAxisGimbal gimble;
    private CachedServo latchServo;
    private Slides slides;
    private RevColorSensorV3 outtakeProximity;
    private RevColorSensorV3 intakeProximity;

    private IndicatorLight light;

    // Internal Subsystem State
    public IntakeState state;
    private double extension = PositionalBounds.SlidePositions.IntakePositions.extended;
    private double tilt = 0;

    // Telemetry
    private Telemetry telemetry;

    private class Slides {

        private CachedMotor motor;
        private double target = 0;
        private PIDController controller;
        private Slides(HardwareMap hwmp) {
            this.controller = new PIDController(
                    core.parameters.pidfCoefficients.IntakeSlides.p,
                    core.parameters.pidfCoefficients.IntakeSlides.i,
                    core.parameters.pidfCoefficients.IntakeSlides.d
            );
            this.controller.setPID(
                    core.parameters.pidfCoefficients.IntakeSlides.p,
                    core.parameters.pidfCoefficients.IntakeSlides.i,
                    core.parameters.pidfCoefficients.IntakeSlides.d
            );
            this.motor = new CachedMotor(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeSlide);
            this.motor.setReversed(HardwareParameters.Motors.Reversed.intakeSlide);
            this.setPosition(PositionalBounds.SlidePositions.IntakePositions.retracted);
            this.motor.resetEncoder();
        }

        private void setPosition(double position) {
            this.target = position;
        }

        private void update() {
            double power = this.controller.calculate(this.getPosition(), this.target);
            this.motor.setPower(power);
        }

        private double getPosition() {
            return this.motor.getPosition();
        }

        private boolean isExtended() {
            return this.getPosition() > extension * 0.8;
        }

        private boolean isRetracted() {
            return this.getPosition() < 50;
        }
    }

    public Intake(HardwareMap hwmp, Telemetry telemetry, IndicatorLight light) {
        this.outtakeProximity = hwmp.get(RevColorSensorV3.class, HardwareParameters.Sensors.HardwareMapNames.outtakeProximity);
        this.intakeProximity = hwmp.get(RevColorSensorV3.class, HardwareParameters.Sensors.HardwareMapNames.intakeProximity);

        this.state = IntakeState.RetractedClawOpen;
        this.telemetry = telemetry;
        this.slides = new Slides(hwmp);

        // Claw and gimble do not need to be scheduled as they are servo abstractions and need no update
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeClawServo);
        this.claw.setState(Subsystems.ClawState.WideOpen);

        this.gimble = new DualAxisGimbal(hwmp,
                HardwareParameters.Motors.HardwareMapNames.leftIntakeLiftServo,
                HardwareParameters.Motors.HardwareMapNames.rightIntakeLiftServo,
                HardwareParameters.Motors.HardwareMapNames.intakeYawServo);

        // Schedule SLIDES, as they must constantly update as they contain a PID controller
        // prevents developer error later by ensuring the subsystem is registered no matter what
        this.gimble.resetPosition();
        this.latchServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.latchServo);
        this.light = light;
    }

    public Intake(HardwareMap hwmp, Telemetry telemetry) {
        if (this.slides != null) telemetry.addData("intakePos", getSlideExtension());
        telemetry.addData("intakeExt", extension);

        this.outtakeProximity = hwmp.get(RevColorSensorV3.class, HardwareParameters.Sensors.HardwareMapNames.outtakeProximity);
        this.intakeProximity = hwmp.get(RevColorSensorV3.class, HardwareParameters.Sensors.HardwareMapNames.intakeProximity);

        this.state = IntakeState.RetractedClawOpen;
        this.telemetry = telemetry;
        this.slides = new Slides(hwmp);

        // Claw and gimble do not need to be scheduled as they are servo abstractions and need no update
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeClawServo);
        this.claw.setState(Subsystems.ClawState.WideOpen);

        this.gimble = new DualAxisGimbal(hwmp,
                HardwareParameters.Motors.HardwareMapNames.leftIntakeLiftServo,
                HardwareParameters.Motors.HardwareMapNames.rightIntakeLiftServo,
                HardwareParameters.Motors.HardwareMapNames.intakeYawServo);

        // Schedule SLIDES, as they must constantly update as they contain a PID controller
        // prevents developer error later by ensuring the subsystem is registered no matter what
        this.gimble.resetPosition();
        this.latchServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.latchServo);
        this.light = null;
    }

    public void nextState() {
        // Internal intake state machine - robot specific
        // Logical flow is as follows:
        // Retracted -> Extended (claw up) -> Extended (claw down) -> Extended (grabbing) -> Transfer -> Retracted (back to start)

        switch (this.state) {
            case RetractedClawOpen:
                this.state = IntakeState.ExtendedClawUp;
                break;
            case ExtendedClawUp:
                this.state = IntakeState.ExtendedClawDown;
                break;
            case ExtendedClawDown:
                this.state = IntakeState.ExtendedClawGrabbing;
                break;
            case ExtendedClawGrabbing:
                this.state = IntakeState.RetractedClawClosed;
                break;
            case RetractedClawClosed:
                this.state = IntakeState.RetractedClawOpen;
                break;
        }

    }

    // Allow for claw to be opened without breaking state machine
    public void cancelGrab() {
        if (this.state == IntakeState.ExtendedClawGrabbing) {
            this.state = IntakeState.ExtendedClawDown;
            this.claw.setState(Subsystems.ClawState.WideOpen);
        }
    }

    public boolean gimblePitchDone() {
        return this.gimble.doneFolding();
    }

    // Return the new yaw, if necessary
    public double rotateIntakeClaw(double speed) {
        this.gimble.rotateYaw(speed);
        return this.gimble.getYaw();
    }

    public void setIntakeClawRotation(double rotation) {
        this.gimble.setYaw(rotation);
    }

    // Allow for possession of nested subclass without owning entire intake class
    public DualAxisGimbal takeGimbleSubsystem() {
        return this.gimble;
    }

    @Override
    public void periodic() {
        telemetry.addData("[PERIODIC] Intake: ", this.state.toString());

        if (pidfCoefficients.IntakeSlides.tuning) {
            this.slides.controller.setPID(
                    pidfCoefficients.IntakeSlides.p,
                    pidfCoefficients.IntakeSlides.i,
                    pidfCoefficients.IntakeSlides.d
            );
        }

        // Indicate transfer status if some light was given to the intake
        // Do not give access to the light if trying to use it for other things
        if (this.light != null) {
            Colour.SampleColour sampleColour = Colour.analyse(intakeProximity);
            switch (sampleColour) {
                case Red:
                    light.setColour(0.28);
                    break;
                case Blue:
                    light.setColour(0.611);
                    break;
                case Yellow:
                    light.setColour(0.388);
                    break;
                case None:
                    light.setColour(0);
                    break;
            }
        }

        switch (this.state) {
            case RetractedClawOpen:
                this.slides.setPosition(PositionalBounds.SlidePositions.IntakePositions.retracted);
                this.claw.setState(Subsystems.ClawState.WideOpen);
                this.gimble.resetPosition();
                break;
            case ExtendedClawUp:
                this.slides.setPosition(extension);
                this.claw.setState(Subsystems.ClawState.WideOpen);
                this.gimble.extendPitch();
                break;
            case ExtendedClawDown:
                this.slides.setPosition(extension);
                this.gimble.extendPitch();
                this.claw.setState(Subsystems.ClawState.WideOpen);
                break;
            case ExtendedClawGrabbing:
                this.slides.setPosition(extension);
                this.gimble.extendPitch();
                if (this.gimblePitchDown()) this.claw.setState(Subsystems.ClawState.WeakGripClosed);
                break;
            case RetractedClawClosed:
                if (this.isSlideLatched()) {
                    claw.setState(Subsystems.ClawState.StrongGripClosed);
                } else {
                    claw.setState(Subsystems.ClawState.WeakGripClosed);
                }
                if (this.gimble.foldedUp()) {
                    this.slides.setPosition(PositionalBounds.SlidePositions.IntakePositions.retracted);
                }
                else {
                    this.slides.setPosition(extension);
                }
                this.gimble.resetPosition();
                break;
        }

        if (this.state == IntakeState.RetractedClawOpen || this.state == IntakeState.RetractedClawClosed) {
            if (this.slides.isRetracted()) this.latchServo.setPosition(PositionalBounds.ServoPositions.LatchPositions.closed);
            else this.latchServo.setPosition(PositionalBounds.ServoPositions.LatchPositions.open);
        } else this.latchServo.setPosition(PositionalBounds.ServoPositions.LatchPositions.open);
        this.slides.update();
    }

    public boolean hasClawClosed() {
        return this.claw.hasClawPhysicallyClosed();
    }

    public boolean isSlideLatched() {
        return (this.state == IntakeState.RetractedClawClosed || this.state == IntakeState.RetractedClawOpen)
                && (this.outtakeProximity.getDistance(DistanceUnit.MM) < PositionalBounds.Sensors.transferThreshold);
    }

    public boolean isSlidesExtended() {
        return this.slides.isExtended();
    }

    public boolean isSlidesPartiallyExtended() {
        return this.slides.getPosition() >= this.extension * 0.5;
    }

    public void setExtension(double extension) { this.extension = extension; }
    public void resetExtension(double extension) { this.extension = PositionalBounds.SlidePositions.IntakePositions.extended; }

    public boolean gimblePitchDown() { return this.gimble.foldedDown(); }
    public boolean clawOpen() {
        return this.claw.hasClawPhysicallyOpened();
    }
    public double getSlideExtension() {
        return this.slides.getPosition();
    }

    public double getExtension() { return this.extension; }

    public void setTilt(double newTilt) {
        this.tilt = newTilt;
        this.gimble.setTilt(tilt);
    }

    public double getTilt() {
        return this.tilt;
    }

    public double getSlidePosition() { return this.slides.getPosition(); }

    public boolean hasIntakeGotSample() {
        return this.intakeProximity.getDistance(DistanceUnit.MM) < PositionalBounds.Sensors.intakeThreshold;
    }

    public boolean isSlidesRetracted() {
        return this.slides.isRetracted();
    }

    public boolean isClawHoveringOverSample() {
        return this.intakeProximity.getDistance(DistanceUnit.MM) < PositionalBounds.Sensors.intakeHovering;
    }
}
