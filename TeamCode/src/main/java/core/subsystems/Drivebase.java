package core.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import core.hardware.CachedMotor;
import core.math.Vector;
import core.parameters.RobotParameters;

public class Drivebase extends SubsystemBase {
    public Vector driveVector;

    public CachedMotor frontRight;
    public CachedMotor frontLeft;
    public CachedMotor backLeft;
    public CachedMotor backRight;

    public Drivebase(HardwareMap hardwareMap) {
        driveVector = Vector.cartesian(0, 0);

        // Create Motors
        frontRight = new CachedMotor(hardwareMap, RobotParameters.Motors.HardwareMapNames.frontRight);
        frontLeft = new CachedMotor(hardwareMap, RobotParameters.Motors.HardwareMapNames.frontLeft);
        backRight = new CachedMotor(hardwareMap, RobotParameters.Motors.HardwareMapNames.backRight);
        backLeft = new CachedMotor(hardwareMap, RobotParameters.Motors.HardwareMapNames.backLeft);

        // Set reversed motors
        frontRight.setReversed(RobotParameters.Motors.Reversed.frontRight);
        frontLeft.setReversed(RobotParameters.Motors.Reversed.frontLeft);
        backRight.setReversed(RobotParameters.Motors.Reversed.backRight);
        backLeft.setReversed(RobotParameters.Motors.Reversed.backLeft);

        // Apply zero power braking
        frontRight.setZeroPowerBehaviour(RobotParameters.Motors.ZeroPowerModes.drivetrain);
        frontLeft.setZeroPowerBehaviour(RobotParameters.Motors.ZeroPowerModes.drivetrain);
        backRight.setZeroPowerBehaviour(RobotParameters.Motors.ZeroPowerModes.drivetrain);
        backLeft.setZeroPowerBehaviour(RobotParameters.Motors.ZeroPowerModes.drivetrain);
    }

    @Override
    public void periodic() {

    }
}
