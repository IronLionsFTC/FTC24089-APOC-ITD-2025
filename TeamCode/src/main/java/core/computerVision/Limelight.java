package core.computerVision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.math.Vector;

public class Limelight {
    private final Limelight3A hardware;

    public Limelight(HardwareMap hwmp) {
        this.hardware = hwmp.get(Limelight3A.class, "limelight");
        this.hardware.pipelineSwitch(0);
        this.hardware.updatePythonInputs(0, 0, 0, 0, 0, 0, 0, 0);
    }

    public void enable() {
        hardware.start();
    }
    public void logStatus(Telemetry telemetry) {
        telemetry.addData("LL3A STATUS", hardware.getStatus().toString());
    }
    public void disable() { hardware.stop(); }
    public void shutdown() { hardware.shutdown(); }

    public void load(int id) {
        this.hardware.pipelineSwitch(id);
    }

    public static class SampleState {
        public double angle;
        public Vector center;

        public SampleState(double angle, Vector center) {
            this.angle = angle;
            this.center = center;
        }
    }

    public SampleState query(Telemetry telemetry) {
        LLResult result = hardware.getLatestResult();

        telemetry.addData("SOMERESULT", result == null);
        if (result == null) return null;

        double[] result_array = result.getPythonOutput();
        telemetry.addData("SOMEPYTHON", result_array == null);

        if (result_array == null) return null;
        if (result_array.length == 0) return null;
        double angle = result_array[0];

        // This is POSSIBLE, but so unlikely it can be treated as an error
        if (angle == 0) return null;

        Vector center = Vector.cartesian(result_array[1], result_array[2]);
        return new SampleState(angle, center);
    }
}
