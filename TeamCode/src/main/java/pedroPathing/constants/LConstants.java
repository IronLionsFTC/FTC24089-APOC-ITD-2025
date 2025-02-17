package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

import core.parameters.HardwareParameters;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.001989436789;
        ThreeWheelConstants.strafeTicksToInches =  0.001989436789;
        ThreeWheelConstants.turnTicksToInches =    0.001989436789;

        ThreeWheelConstants.leftY = HardwareParameters.Odometry.CenterOffset_in.lefty;
        ThreeWheelConstants.rightY = HardwareParameters.Odometry.CenterOffset_in.righty;
        ThreeWheelConstants.strafeX = HardwareParameters.Odometry.CenterOffset_in.sidex;

        ThreeWheelConstants.leftEncoder_HardwareMapName = HardwareParameters.Odometry.HardwareMapNames.left;
        ThreeWheelConstants.rightEncoder_HardwareMapName = HardwareParameters.Odometry.HardwareMapNames.right;
        ThreeWheelConstants.strafeEncoder_HardwareMapName = HardwareParameters.Odometry.HardwareMapNames.sideways;

        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




