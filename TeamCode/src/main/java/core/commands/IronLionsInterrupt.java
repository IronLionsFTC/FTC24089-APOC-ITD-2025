package core.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class IronLionsInterrupt extends CommandBase {
    private Command command;
    private BooleanSupplier condition;
    private boolean done;

    public IronLionsInterrupt(Command command, BooleanSupplier condition) {
        this.command = command;
        this.condition = condition;
        this.done = false;
    }

    @Override
    public void initialize() {
        this.command.initialize();
    }

    @Override
    public void execute() {
        this.command.execute();
        if (this.condition.getAsBoolean()) {
            this.command.end(false);
            this.done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (this.done || this.command.isFinished());
    }
}
