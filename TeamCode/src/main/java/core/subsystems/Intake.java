package core.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.hardware.MasterSlaveMotorPair;
import core.parameters.HardwareParameters;
import core.parameters.pidfCoefficients;
import core.state.Subsystems;

public class Intake extends SubsystemBase {

    // Subsystems of intake
    private LinearSlides slides;
    private Claw claw;
    private DualAxisGimble gimble;

    // Internal Subsystem State
    public Subsystems.IntakeState state = Subsystems.IntakeState.RetractedClawOpen;

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

        // Claw and gimble do not need to be scheduled as they are servo abstractions and need no update
        this.claw = new Claw(hwmp, HardwareParameters.Motors.HardwareMapNames.intakeClawServo);
        this.gimble = new DualAxisGimble(hwmp,
                HardwareParameters.Motors.HardwareMapNames.intakeLiftServo,
                HardwareParameters.Motors.HardwareMapNames.intakeYawServo);

        // Schedule SLIDES, as they must constantly update as they contain a PID controller
        // prevents developer error later by ensuring the subsystem is registered no matter what
        CommandScheduler.getInstance().registerSubsystem(this.slides);
    }

    public void nextState() {
        // Internal intake state machine - robot specific
        // Logical flow is as follows:
        // Retracted -> Extended (claw up) -> Extended (claw down) -> Extended (grabbing) -> Transfer -> Retracted (back to start)

        switch (this.state) {
            case RetractedClawOpen:
                this.state = Subsystems.IntakeState.ExtendedClawUp;
                this.slides.setTarget(1);
                break;
            case ExtendedClawUp:
                this.state = Subsystems.IntakeState.ExtendedClawDown;
                this.gimble.extendPitch();
                break;
            case ExtendedClawDown:
                this.state = Subsystems.IntakeState.ExtendedClawGrabbing;
                this.claw.setState(Subsystems.ClawState.WeakGripClosed);
                break;
            case ExtendedClawGrabbing:
                this.state = Subsystems.IntakeState.RetractedClawClosed;
                this.gimble.resetPosition();
                this.slides.setTarget(0);
                break;
            default:
                this.state = Subsystems.IntakeState.RetractedClawOpen;
                this.claw.setState(Subsystems.ClawState.WideOpen);
                break;
        }
    }

    // Allow for claw to be opened without breaking state machine
    public void cancelGrab() {
        if (this.state == Subsystems.IntakeState.ExtendedClawGrabbing) {
            this.state = Subsystems.IntakeState.ExtendedClawDown;
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
}
