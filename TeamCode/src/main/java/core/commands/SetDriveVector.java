package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import core.math.Vector;
import core.subsystems.Drivebase;
import core.subsystems.Intake;

public class SetDriveVector extends CommandBase {
    private final Drivebase drivebaseSubsystem;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier r;
    private double power;
    private Intake intakeSubsystem;
    private Vector driveVector;

    public SetDriveVector(Drivebase drivebaseSubsystem, Intake intakeSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.power = 1;
        this.x = x; this.y = y; this.r = r;
    }

    private void calculateDriveVector() {

        double xVal = this.x.getAsDouble();
        double yVal = this.y.getAsDouble();

        this.driveVector = Vector.cartesian(
                Math.pow(xVal, 3) * 1.1,
                Math.pow(yVal, 3) * -1.1
        );
    }

    @Override
    public void initialize() {
        this.calculateDriveVector();
    }

    @Override
    public void execute() {

        this.power = 1;

        this.calculateDriveVector();
        this.drivebaseSubsystem.setDriveVector(this.driveVector.mul(this.power));
        this.drivebaseSubsystem.setYawInput(Math.pow(this.r.getAsDouble(), 3));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}