package core.math;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import core.parameters.PositionalBounds;

public class Colour {
    public enum SampleColour {
        Yellow,
        Blue,
        Red,
        None
    }

    public static SampleColour analyse(RevColorSensorV3 sensor) {
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();

        if (r > g + b) { return SampleColour.Red; }
        if (b > g + r) { return SampleColour.Blue; }
        else {
            if (sensor.getDistance(DistanceUnit.MM) < PositionalBounds.Sensors.intakeHovering) {
                return SampleColour.Yellow;
            } else {
                return SampleColour.None;
            }
        }
    }
}
