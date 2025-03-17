package core.computerVision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.math.Vector;

public class Limelight {
    private final Limelight3A hardware;

    public Limelight(HardwareMap hwmp) {
        this.hardware = hwmp.get(Limelight3A.class, "limelight");
    }

    public void enable() { hardware.start(); }
    public void disable() { hardware.stop(); }
    public void shutdown() { hardware.shutdown(); }

    public class SampleState {
        public double angle;
        public Vector center;

        public SampleState(double angle, Vector center) {
            this.angle = angle;
            this.center = center;
        }
    }

    public SampleState query() {
        LLResult result = hardware.getLatestResult();

        if (result == null) return null;

        double[] result_array = result.getPythonOutput();
        double angle = result_array[0];

        // This is POSSIBLE, but so unlikely it can be treated as an error
        if (angle == 0) return null;

        Vector center = Vector.cartesian(result_array[1], result_array[2]);
        return new SampleState(angle, center);
    }
}
