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
        this.driveVector = Vector.cartesian(this.x.getAsDouble(), -this.y.getAsDouble());
    }

    @Override
    public void initialize() {
        this.calculateDriveVector();
    }

    @Override
    public void execute() {

        if (this.intakeSubsystem.isSlideLatched()) { this.power = 1; }
        else { this.power = 0.7; }

        this.calculateDriveVector();
        this.drivebaseSubsystem.setDriveVector(this.driveVector.mul(this.power));
        this.drivebaseSubsystem.setYawInput(this.r.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}