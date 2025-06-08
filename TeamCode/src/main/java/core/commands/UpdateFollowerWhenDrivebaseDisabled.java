package core.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import java.util.function.BooleanSupplier;

import core.subsystems.Drivebase;

public class UpdateFollowerWhenDrivebaseDisabled extends CommandBase {
    private Drivebase drivebase;
    private Follower follower;
    private BooleanSupplier joystick;

    public UpdateFollowerWhenDrivebaseDisabled(Drivebase drivebase, Follower follower, BooleanSupplier joystick) {
        this.drivebase = drivebase;
        this.follower = follower;
        this.joystick = joystick;
    }

    @Override
    public void execute() {
        if (this.joystick.getAsBoolean()) this.drivebase.enable();
        if (!this.drivebase.active()) follower.update();
    }
}
