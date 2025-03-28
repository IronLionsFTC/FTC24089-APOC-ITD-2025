package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import core.hardware.CachedServo;
import core.parameters.HardwareParameters;
import core.parameters.PositionalBounds;
import core.state.Subsystems;
import core.state.Subsystems.IntakeState;

public class Intake extends SubsystemBase {

    // Subsystems of intake
    private Claw claw;
    private DualAxisGimbal gimble;
    private CachedServo latchServo;
    private Slides slides;
    private RevColorSensorV3 outtakeProximity;

    // Internal Subsystem State
    public IntakeState state;
    private int retractionCounter;

    // Telemetry
    private Telemetry telemetry;

    private class Slides {
        private CachedServo leftSlide;
        private CachedServo rightSlide;

        private Slides(HardwareMap hwmp) {
            this.leftSlide = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.leftIntakeServo);
            this.rightSlide = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.rightIntakeServo);
            this.setPosition(PositionalBounds.SlidePositions.IntakePositions.retracted);
        }

        private void setPosition(double position) {
            this.leftSlide.setPosition(position);
            this.rightSlide.setPosition(1 - position);
        }

        private double getPosition() {
            return this.leftSlide.getPosition();
        }

        private boolean isExtended() {
            return this.getPosition() == PositionalBounds.SlidePositions.IntakePositions.extended && this.leftSlide.secondsSinceMovement() > 0.5;
        }

        private boolean isRetracted() {
            return this.getPosition() == PositionalBounds.SlidePositions.IntakePositions.retracted && this.leftSlide.secondsSinceMovement() > 1.5;
        }
    }

    public Intake(HardwareMap hwmp, Telemetry telemetry) {
        this.outtakeProximity = hwmp.get(RevColorSensorV3.class, HardwareParameters.Sensors.HardwareMapNames.outtakeProximity);
        this.state = IntakeState.RetractedClawOpen;
        this.retractionCounter = 0;
        this.telemetry = telemetry;
        this.slides = new Slides(hwmp);

        // Claw and gimble do not need to be scheduled as they are servo abstractions and need no update
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeClawServo);
        this.claw.setState(Subsystems.ClawState.WideOpen);

        this.gimble = new DualAxisGimbal(hwmp,
                HardwareParameters.Motors.HardwareMapNames.intakeLiftServo,
                HardwareParameters.Motors.HardwareMapNames.intakeYawServo);

        // Schedule SLIDES, as they must constantly update as they contain a PID controller
        // prevents developer error later by ensuring the subsystem is registered no matter what
        this.gimble.resetPosition();
        this.latchServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.latchServo);
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
        switch (this.state) {
            case RetractedClawOpen:
                this.slides.setPosition(PositionalBounds.SlidePositions.IntakePositions.retracted);
                this.claw.setState(Subsystems.ClawState.WideOpen);
                this.gimble.resetPosition();
                break;
            case ExtendedClawUp:
                this.slides.setPosition(PositionalBounds.SlidePositions.IntakePositions.extended);
                this.claw.setState(Subsystems.ClawState.WideOpen);
                this.gimble.resetPosition();
                break;
            case ExtendedClawDown:
                this.slides.setPosition(PositionalBounds.SlidePositions.IntakePositions.extended);
                this.gimble.extendPitch();
                this.claw.setState(Subsystems.ClawState.WideOpen);
                break;
            case ExtendedClawGrabbing:
                this.slides.setPosition(PositionalBounds.SlidePositions.IntakePositions.extended);
                this.gimble.extendPitch();
                this.claw.setState(Subsystems.ClawState.WeakGripClosed);
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
                    this.slides.setPosition(PositionalBounds.SlidePositions.IntakePositions.extended);
                }
                this.gimble.resetPosition();
                break;
        }

        if (this.state == IntakeState.RetractedClawOpen || this.state == IntakeState.RetractedClawClosed) {
            if (this.slides.isRetracted()) this.latchServo.setPosition(PositionalBounds.ServoPositions.LatchPositions.closed);
            else this.latchServo.setPosition(PositionalBounds.ServoPositions.LatchPositions.open);
        } else this.latchServo.setPosition(PositionalBounds.ServoPositions.LatchPositions.open); }

    public boolean hasClawClosed() {
        return this.claw.hasClawPhysicallyClosed();
    }

    public boolean isSlideLatched() {
        return (this.state == IntakeState.RetractedClawClosed || this.state == IntakeState.RetractedClawOpen)
                && (this.outtakeProximity.getDistance(DistanceUnit.MM) < PositionalBounds.Sensors.transferThreshold || this.slides.isRetracted());
    }

    public boolean isSlidesExtended() {
        return this.slides.isExtended();
    }

    public boolean isSlidesPartiallyExtended() {
        return this.slides.getPosition() >= PositionalBounds.SlidePositions.IntakePositions.extended && this.slides.leftSlide.secondsSinceMovement() > 0.25;
    }

    public boolean gimblePitchDown() { return this.gimble.foldedDown(); }
    public boolean clawOpen() {
        return this.claw.hasClawPhysicallyOpened();
    }
}
