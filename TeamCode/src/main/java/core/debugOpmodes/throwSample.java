package core.debugOpmodes;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.hardware.CachedServo;
import core.parameters.HardwareParameters;

@TeleOp(name = "Throw Sample!")
public class throwSample extends LinearOpMode {

    private CachedServo leftArmServo;
    private CachedServo rightArmServo;
    private CachedServo pitchServo;
    private CachedServo clawServo;

    private Timer timer;

    private int buttonCounter;
    private int state;

    private void setArmPosition(double position) {
        this.leftArmServo.setPosition(position);
        this.rightArmServo.setPosition(1 - position);
    }

    private void grab() { this.clawServo.setPosition(0); }
    private void release() { this.clawServo.setPosition(0.5); }

    @Override
    public void runOpMode() {

        // Load servos
        leftArmServo = new CachedServo(hardwareMap, HardwareParameters.Motors.HardwareMapNames.leftArmServo);
        rightArmServo = new CachedServo(hardwareMap, HardwareParameters.Motors.HardwareMapNames.rightArmServo);
        pitchServo = new CachedServo(hardwareMap, HardwareParameters.Motors.HardwareMapNames.outtakePitchServo);
        clawServo = new CachedServo(hardwareMap, HardwareParameters.Motors.HardwareMapNames.outtakeClawServo);

        timer = new Timer();

        setArmPosition(0);
        pitchServo.setPosition(0.3);

        buttonCounter = 0;
        state = 0;

        if (isStopRequested()) return;
        waitForStart();

        while (this.opModeIsActive()) {
            switch (state) {
                case 1:
                    setArmPosition(0);
                    grab();
                    break;
                case 2:
                    setArmPosition(0.6);
                    grab();
                    break;
                case 3:
                    setArmPosition(0);
                    if (timer.getElapsedTimeSeconds() < 0.35) grab();
                    else state = 0;
                    break;
                default:
                    setArmPosition(0);
                    release();
                    state = 0;
                    break;
            }

            if (gamepad1.x) buttonCounter += 1;
            else buttonCounter = 0;

            if (buttonCounter == 1) {
                state += 1;
                timer.resetTimer();
            }
        }
    }
}
