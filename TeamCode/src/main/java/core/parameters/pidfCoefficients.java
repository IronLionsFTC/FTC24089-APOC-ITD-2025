package core.parameters;

import com.acmerobotics.dashboard.config.Config;

public class pidfCoefficients {

    @Config
    public static final class Drivetrain {
        public static double p = 0;
        public static double i = 0;
        public static double d = 0;
        public static double f = 0;
        public static boolean tuning = false;
    }
}
