package core.computerVision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    private final Limelight3A hardware;

    public Limelight(HardwareMap hwmp) {
        this.hardware = hwmp.get(Limelight3A.class, "limelight");
    }
}
