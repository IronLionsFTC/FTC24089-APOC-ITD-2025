package core.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
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
import core.state.Subsystems.IntakeState;

public class Intake extends SubsystemBase {

    // Subsystems of intake
    private LinearSlides slides;
    private Claw claw;
    private DualAxisGimble gimble;
    private CachedServo latchServo;

    // Internal Subsystem State
    public IntakeState state;
    private int retractionCounter;

    // Hardware Interface / Controllers
    private PIDController slideController;
    private MasterSlaveMotorPair slideMotors;

    // Telemetry
    private Telemetry telemetry;

    public Intake(HardwareMap hwmp, Telemetry telemetry) {
        this.state = IntakeState.RetractedClawOpen;
        this.retractionCounter = 0;
        this.telemetry = telemetry;
        this.slideMotors = new MasterSlaveMotorPair(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeSlide, HardwareParameters.Motors.Reversed.intakeSlide);
        this.slideController = new PIDController(
                pidfCoefficients.IntakeSlides.p,
                pidfCoefficients.IntakeSlides.i,
                pidfCoefficients.IntakeSlides.d
        );
        this.slides = new LinearSlides(this.slideMotors, this.slideController, this.telemetry, pidfCoefficients.IntakeSlides.f, 145);
        this.slides.setTarget(0);

        // Claw and gimble do not need to be scheduled as they are servo abstractions and need no update
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeClawServo);
        this.gimble = new DualAxisGimble(hwmp,
                HardwareParameters.Motors.HardwareMapNames.intakeLiftServo,
                HardwareParameters.Motors.HardwareMapNames.intakeYawServo);

        // Schedule SLIDES, as they must constantly update as they contain a PID controller
        // prevents developer error later by ensuring the subsystem is registered no matter what
        this.gimble.resetPosition();
        this.latchServo = new CachedServo(hwmp, HardwareParameters.Motors.HardwareMapNames.latchServo);
        CommandScheduler.getInstance().registerSubsystem(this.slides);
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

    public double slideExtension() {
        return this.slides.getRelative();
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
    public DualAxisGimble takeGimbleSubsystem() {
        return this.gimble;
    }

    @Override
    public void periodic() {
        if (pidfCoefficients.IntakeSlides.tuning) {
            this.slideController.setPID(
                pidfCoefficients.IntakeSlides.p,
                pidfCoefficients.IntakeSlides.i,
                pidfCoefficients.IntakeSlides.d
            );
        }

        // Track retraction stability (make sure it hasn't bounced out)
        if (this.slideExtension() < 0.05) retractionCounter += 1;
        else retractionCounter = 0;

        switch (this.state) {
            case RetractedClawOpen:
                this.slides.setTarget(0);
                this.claw.setState(Subsystems.ClawState.WideOpen);
                this.gimble.resetPosition();
                break;
            case ExtendedClawUp:
                this.slides.setTarget(1);
                this.claw.setState(Subsystems.ClawState.WideOpen);
                this.gimble.resetPosition();
                break;
            case ExtendedClawDown:
                this.slides.setTarget(1);
                this.gimble.extendPitch();
                this.claw.setState(Subsystems.ClawState.WideOpen);
                break;
            case ExtendedClawGrabbing:
                this.slides.setTarget(1);
                this.gimble.extendPitch();
                this.claw.setState(Subsystems.ClawState.WeakGripClosed);
                break;
            case RetractedClawClosed:
                this.claw.setState(Subsystems.ClawState.StrongGripClosed);
                if (this.gimble.foldedUp()) { this.slides.setTarget(0); }
                else { this.slides.setTarget(1); }
                this.gimble.resetPosition();
                break;
        }

        if (this.state == IntakeState.RetractedClawOpen || this.state == IntakeState.RetractedClawClosed) {
            if (retractionCounter > 0) this.latchServo.setPosition(PositionalBounds.ServoPositions.LatchPositions.closed);
            else this.latchServo.setPosition(PositionalBounds.ServoPositions.LatchPositions.open);
        } else this.latchServo.setPosition(PositionalBounds.ServoPositions.LatchPositions.open); }
    public boolean hasClawClosed() {
        return this.claw.hasClawPhysicallyClosed();
    }

    public boolean isSlideLatched() {
        return this.retractionCounter > 5 && (this.state == IntakeState.RetractedClawClosed || this.state == IntakeState.RetractedClawOpen);
    }
}
