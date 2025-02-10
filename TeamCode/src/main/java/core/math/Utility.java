package core.math;

public class Utility {
    public static double clamp(double value, double minimum, double maximum) {
        if (value < minimum) { return minimum; }
        return Math.min(value, maximum);
    }
}
