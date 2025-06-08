package core.parameters;

public final class PositionalBounds {

    // Servo positions, 0-1.
    public static final class ServoPositions {

        public static final double limelightDown = 0;
        public static final double limelightUp = 0.93;

        public static final class ClawPositions {

            // Claw positions
            public static final double strongGripPosition = 0;
            public static final double weakGripPosition = 0.02;
            public static final double openPosition = 0.5;
            public static final double wideOpenPosition = 0.65;

            // Gimble positions
            public static final double pitchRest = 0;
            public static final double pitchExtended = 0.5;
            public static final double yawRest = 0.465;
            // How far in each direction can the servo turn from the
            // yaw rest
            public static final double yawRange = 0.5;
        }

        public static final class LatchPositions {
            public static final double open = 0.13;
            public static final double closed = 0.4;
        }

        public static final class Outtake {
            public static final double armDown = 0.0;
            public static final double armSample = 0.2;
            public static final double armSpecimen = 0.5;

            public static final double pitchSampleOuttake = 0.05;
            public static final double pitchSpecimenIntake = 0.05;
            public static final double pitchSampleTransfer = 0.95;
            public static final double safeMovement = 0.5;

            public static final double specimenEntry = 0.6;
        }
    }

    public static final class SlidePositions {

        // Degrees of motor rotation for maximum extension,
        // Positions expressed as relative 0-1 so that they are consistent
        // Across motors and ratios

        public static final double outtakeMaximumExtension = 730;

        public static final class OuttakePositions {
            public static final double highBasket = 1;
            public static final double lowBasket = 0.2;
            public static final double specimenOuttake = 0.35;
        }

        public static final class IntakePositions {
            public static final double retracted = 0;
            public static final double extended = 697;
        }
    }

    public static final class Sensors {
        public static final double transferThreshold = 18;
        public static final double intakeThreshold = 15;
        public static final double intakeHovering = 28;
    }
}