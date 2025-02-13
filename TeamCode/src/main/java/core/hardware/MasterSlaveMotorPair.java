package core.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class MasterSlaveMotorPair {
    private final CachedMotor master;
    private final CachedMotor slave; // Slave MAY be NULL
    private final boolean hasSlave;

    // Expose a simple constructor, allowing for motors to be referenced as a pair without ownership
    public MasterSlaveMotorPair(CachedMotor master, CachedMotor slave) {
        this.master = master;
        this.slave = slave;
        this.hasSlave = true;

        this.master.resetEncoder();
    }

    // Allow for construction of motors owned by the pair
    public MasterSlaveMotorPair(HardwareMap hwmp, String master, boolean reverseMaster, String slave, boolean reverseSlave) {
        this.master = new CachedMotor(hwmp, master);
        this.master.setReversed(reverseMaster);
        this.slave = new CachedMotor(hwmp, slave);
        this.slave.setReversed(reverseSlave);
        this.hasSlave = true;

        this.master.resetEncoder();
    }

    // Master without slave (single motor wrapped in pair for functions that assume a pair).
    // this is allowed by the master / slave architecture, because the slave exists only
    // to supplement the power of the master, encoder position is read from master
    public MasterSlaveMotorPair(CachedMotor master) {
        this.master = master;
        this.slave = null;
        this.hasSlave = false;

        this.master.resetEncoder();
    }

    public MasterSlaveMotorPair(HardwareMap hwmp, String master, boolean reverseMaster) {
        this.master = new CachedMotor(hwmp, master);
        this.master.setReversed(reverseMaster);
        this.slave = null;
        this.hasSlave = false;

        this.master.resetEncoder();
    }

    // Only update the slaves power if the masters power changed.
    public void setPower(double power) {
        if (this.master.setPower(power) && this.hasSlave) { this.slave.setPower(power); }
    }

    // Encoder position
    public double getPosition() {
        return this.master.getPosition();
    }

    public void resetEncoder() {
        this.master.resetEncoder();
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
