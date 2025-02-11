package core.parameters;

public final class PositionalBounds {

    // Servo positions, 0-1.
    public static final class ServoPositions {

        public static final class ClawPositions {

            // Claw positions
            public static final double strongGripPosition = 0;
            public static final double weakGripPosition = 0.05;
            public static final double openPosition = 0.4;
            public static final double wideOpenPosition = 0.5;

            // Gimble positions
            public static final double pitchRest = 1;
            public static final double pitchExtended = 0;
            public static final double yawRest = 0.5;
            // How far in each direction can the servo turn from the
            // yaw rest
            public static final double yawRange = 0.5;
        }
    }
}
