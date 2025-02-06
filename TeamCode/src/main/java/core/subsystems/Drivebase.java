package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import core.math.Vector;

public class Drivebase extends SubsystemBase {
    public Vector driveVector;

    public MotorEx frontRight;
    public MotorEx frontLeft;
    public MotorEx backLeft;
    public MotorEx backRight;

    public Drivebase() {
        driveVector = Vector.cartesian(0, 0);
    }

    @Override
    public void periodic() {

    }
}
