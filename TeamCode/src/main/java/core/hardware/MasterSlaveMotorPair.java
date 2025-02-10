package core.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class MasterSlaveMotorPair {
    private CachedMotor master;
    private CachedMotor slave; // Slave MAY be NULL
    private boolean hasSlave;

    // Expose a simple constructor, allowing for motors to be referenced as a pair without ownership
    public MasterSlaveMotorPair(CachedMotor master, CachedMotor slave) {
        this.master = master;
        this.slave = slave;
        this.hasSlave = true;
    }

    // Allow for construction of motors owned by the pair
    public MasterSlaveMotorPair(HardwareMap hwmp, String master, String slave) {
        this.master = new CachedMotor(hwmp, master);
        this.slave = new CachedMotor(hwmp, slave);
        this.hasSlave = true;
    }

    // Slaveless master (single motor wrapped in pair for functions that assume a pair).
    // this is allowed by the master / slave architecture, because the slave exists only
    // to supplement the power of the master, encoder positin is read from master
    public MasterSlaveMotorPair(CachedMotor master) {
        this.master = master;
        this.slave = null;
        this.hasSlave = false;
    }

    public MasterSlaveMotorPair(HardwareMap hwmp, String master) {
        this.master = new CachedMotor(hwmp, master);
        this.slave = null;
        this.hasSlave = false;
    }

    // Only update the slaves power if the masters power changed.
    public void setPower(double power) {
        if (this.master.setPower(power) && this.hasSlave) { this.slave.setPower(power); }
    }

    // Encoder position
    public double getPosition() {
        return this.master.getPosition();
    }

    // Despite the fact that it contradicts the master slave paradigm, allow for the average position to be collected
    // This should not be used in PID(f) controllers as it allows for situations (encoder slip) where the motors may fight against
    // their physical bounds (ie for outtake, could create a slant at top).
    public double getMasterSlaveAveragePosition() {
        if (this.hasSlave) {
            return (this.getPosition() + this.slave.getPosition()) * 0.5;
        } else {
            return this.getPosition();
        }
    }
}
