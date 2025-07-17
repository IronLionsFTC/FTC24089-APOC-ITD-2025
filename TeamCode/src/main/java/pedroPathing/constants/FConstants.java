package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import core.parameters.HardwareParameters;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        FollowerConstants.leftFrontMotorName = HardwareParameters.Motors.HardwareMapNames.frontLeft;
        FollowerConstants.leftRearMotorName = HardwareParameters.Motors.HardwareMapNames.backLeft;
        FollowerConstants.rightFrontMotorName = HardwareParameters.Motors.HardwareMapNames.frontRight;
        FollowerConstants.rightRearMotorName = HardwareParameters.Motors.HardwareMapNames.backRight;

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        FollowerConstants.mass = 13.7;

        FollowerConstants.xMovement = 69.92;
        FollowerConstants.yMovement = 46.29;

        FollowerConstants.forwardZeroPowerAcceleration = -34.97;
        FollowerConstants.lateralZeroPowerAcceleration = -99.02;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.5,0,0.05,0);
        //FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.5,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(4,0,0.3,0);
        //FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.001,0.6,0.05);
        //FollowerConstants.drivePIDFCoefficients.setCoefficients(0.025,0,0.002,0.2,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01,0,0,0,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 7;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
