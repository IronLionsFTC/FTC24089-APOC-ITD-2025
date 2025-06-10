package core.commands;

import android.sax.StartElementListener;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import core.computerVision.Limelight;
import core.math.Kinematics;
import core.math.Vector;
import core.state.Subsystems;
import core.subsystems.Intake;

public class DriveToSampleUseSlides extends CommandBase {
    private Follower follower;
    private Intake intakeSubsystem;
    private Limelight.SampleState buffer;
    private final Telemetry telemetry;

    public DriveToSampleUseSlides(Follower follower, Intake intakeSubsystem, Limelight.SampleState buffer, Telemetry telemetry) {
        this.follower = follower;
        this.intakeSubsystem = intakeSubsystem;
        this.buffer = buffer;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        this.follower.setMaxPower(0.8);

        /*
            Calculates the following:
            - Sample position on XZ plane regardless of limelight angle
            -   ^ Center of sample, homography postcalculation dynamically
            - Robot position at the time of seeing a sample, even if it has since moved
            - Robot rotation at the time of seeing a sample
            - The robot movement required to go from its position when it saw the sample to the position of the sample
            - Figure out how much of that can be done without moving the robot, just using slides
            - Calculate required slide, claw, and drivetrain movement
        */
        Kinematics kinematics = new Kinematics(buffer);

        /*
            Calculate the path from the robots current position to that of the sample.
        */
        follower.followPath(kinematics.instantPath(follower), true);
        intakeSubsystem.state = Subsystems.IntakeState.ExtendedClawDown;
        intakeSubsystem.setExtension(kinematics.absoluteSlidePosition);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return follower.getCurrentTValue() > 0.85 && this.intakeSubsystem.isSlidesExtended();
    }

    @Override
    public void end(boolean i) {
        this.follower.setMaxPower(1);
        buffer.center = Vector.cartesian(0, 0);
        buffer.robotPosition = Vector.cartesian(0, 0);
        buffer.robotRotation = 0;
        buffer.slidePosition = 0;
        buffer.intakeTilt = 0;
    }
}
