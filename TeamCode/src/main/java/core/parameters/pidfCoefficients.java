package core.parameters;

import com.acmerobotics.dashboard.config.Config;

public class pidfCoefficients {

    @Config
    public static class Drivetrain {
        public static double p = 0.04;
        public static double i = 0;
        public static double d = 0.002;
        public static double f = 0;
        public static boolean tuning = false;
    }

    @Config
    public static class IntakeExtension {
        public static double p = 0.01;
        public static double i = 0;
        public static double d = 0.00075;
        public static double f = 0;
        public static boolean tuning = false;
    }

    @Config
    public static class IntakeRetraction {
        public static double p = 0.015;
        public static double i = 0;
        public static double d = 0;
        public static boolean tuning = false;
    }

    @Config
    public static class OuttakeSlides {
        public static double p = 0.003;
        public static double i = 0;
        public static double d = 0.0001;
        public static double feedforward = 0.3;
        public static double feedbackward = 0;
        public static boolean tuning = false;
    }
}
