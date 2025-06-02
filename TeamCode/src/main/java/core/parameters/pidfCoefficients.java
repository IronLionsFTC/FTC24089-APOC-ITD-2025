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
    public static class IntakeSlides {
        public static double p = 0.01;
        public static double i = 0;
        public static double d = 0.00055;
        public static double f = 0;
        public static boolean tuning = false;
    }

    @Config
    public static class OuttakeSlides {
        public static double p = 0.006;
        public static double i = 0;
        public static double d = 0;
        public static double feedforward = 0;
        public static double feedbackward = 0;
        public static boolean tuning = false;
    }
}
