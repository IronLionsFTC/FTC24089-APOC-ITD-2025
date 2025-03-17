package core.computerVision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    private final Limelight3A hardware;

    public Limelight(HardwareMap hwmp) {
        this.hardware = hwmp.get(Limelight3A.class, "limelight");
    }

    public void enable() { hardware.start(); }
    public void disable() { hardware.stop(); }
    public void shutdown() { hardware.shutdown(); }

    public void query() {
        LLResult result = hardware.getLatestResult();
        result.getPythonOutput();
    }
}
