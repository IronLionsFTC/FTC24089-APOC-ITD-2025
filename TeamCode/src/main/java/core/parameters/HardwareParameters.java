package core.parameters;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public final class HardwareParameters {
    private HardwareParameters() {}

    public static final class Motors {
        public static final class HardwareMapNames {

            // Drivetrain
            public static final String frontLeft = "frontLeft";
            public static final String backLeft = "backLeft";
            public static final String frontRight = "frontRight";
            public static final String backRight = "backRight";

            // Slide motors
            public static final String intakeSlide = "intakeSlide";
            public static final String leftOuttakeSlide = "leftOuttakeSlide";
            public static final String rightOuttakeSlide = "rightOuttakeSlide";
            public static final String latchServo = "latchServo";

            // Intake Servos
            public static final String intakeLiftServo = "intakeLiftServo"; // Port c0
            public static final String intakeYawServo = "intakeYawServo"; // Port c1
            public static final String intakeClawServo = "intakeClawServo"; // Port c2

            // Outtake Servos
            public static final String outtakeClawServo = "outtakeClawServo";
            public static final String leftArmServo = "leftArmServo";
            public static final String rightArmServo = "rightArmServo";
        }

        public static final class Reversed {

            // Drivetrain
            public static final boolean frontLeft = true;
            public static final boolean backLeft = true;
            public static final boolean frontRight = false;
            public static final boolean backRight = false;

            // Intake Slides
            public static final boolean intakeSlide = true;

            // Outtake Slides
            public static final boolean leftOuttakeSlide = false;
            public static final boolean rightOuttakeSlide = true;
        }


        public static final class ZeroPowerModes {

            // Drivetrain
            public static final Motor.ZeroPowerBehavior drivetrain = Motor.ZeroPowerBehavior.BRAKE;

            // Intake Slides
            public static final Motor.ZeroPowerBehavior intakeSlide = Motor.ZeroPowerBehavior.BRAKE;

            // Outtake Slides
            public static final Motor.ZeroPowerBehavior outtakeSlides = Motor.ZeroPowerBehavior.FLOAT;
        }
    }

    public static final class Odometry {

        // Which encoders are bound to which motors
        public static final class HardwareMapNames {
            public static final String left = HardwareParameters.Motors.HardwareMapNames.backRight;
            public static final String right = HardwareParameters.Motors.HardwareMapNames.frontLeft;
            public static final String sideways = HardwareParameters.Motors.HardwareMapNames.backLeft;
        }

        // Which encoders are reversed
        public static final class Reversed {
            public static final boolean left = true;
            public static final boolean right = true;
            public static final boolean sideways = true;
        }

        // Positions of encoders relative to center of robot
        // in pedro coordinate system
        public static final class CenterOffset_mm {

            // See diagram in LOCALIZATION.md
            // LEFT: 153mm to the LEFT, 105mm forward
            // RIGHT: 153mm to the RIGHT, 105mm forward
            // SIDEWAYS [TEMPORARY, WE MIGHT CHANGE THIS]: 110mm forward, 26mm RIGHT

            public static final double leftx = 105;
            public static final double lefty = 153;
            public static final double rightx = 105;
            public static final double righty = -153;
            public static final double sidex = 120;
            public static final double sidey = 65;
        }

        // Calculate the inches from the mm
        public static final class CenterOffset_in {
            public static final double leftx = HardwareParameters.Odometry.CenterOffset_mm.leftx / 25.4;
            public static final double lefty = HardwareParameters.Odometry.CenterOffset_mm.lefty  / 25.4;
            public static final double rightx = HardwareParameters.Odometry.CenterOffset_mm.rightx  / 25.4;
            public static final double righty = HardwareParameters.Odometry.CenterOffset_mm.righty  / 25.4;
            public static final double sidex = HardwareParameters.Odometry.CenterOffset_mm.sidex  / 25.4;
            public static final double sidey = HardwareParameters.Odometry.CenterOffset_mm.sidey  / 25.4;
        }
    }
}