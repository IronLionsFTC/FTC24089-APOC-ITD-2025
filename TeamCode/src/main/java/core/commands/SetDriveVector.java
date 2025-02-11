package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import core.math.Vector;
import core.subsystems.Drivebase;

public class SetDriveVector extends CommandBase {
    private final Drivebase drivebaseSubsystem;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private Vector driveVector;

    public SetDriveVector(Drivebase drivebaseSubsystem, DoubleSupplier x, DoubleSupplier y) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        this.x = x; this.y = y;
    }

    private void calculateDriveVector() {
        this.driveVector = Vector.cartesian(this.x.getAsDouble(), this.y.getAsDouble());
    }

    @Override
    public void initialize() {
        this.calculateDriveVector();
    }

    @Override
    public void execute() {
        this.calculateDriveVector();
        this.drivebaseSubsystem.setDriveVector(this.driveVector);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}