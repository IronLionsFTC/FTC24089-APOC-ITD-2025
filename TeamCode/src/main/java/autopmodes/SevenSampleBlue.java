package autopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import core.commands.CMD;
import core.computerVision.Limelight;
import core.hardware.IndicatorLight;
import core.math.Vector;
import core.paths.PathMaker;
import core.subsystems.Intake;
import core.subsystems.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous ( name = "- 7 Sample [BLUE] -" )
public class SevenSampleBlue extends CommandOpMode {

    private Intake intakeSubsystem;
    private Outtake outtakeSubsystem;
    private Follower follower;

    private Limelight limelight;
    private Limelight.SampleState buffer;
    private Limelight.SampleState cache;

    private IndicatorLight light;
    private PathMaker makeDump;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intakeSubsystem = new Intake(hardwareMap, this.telemetry);
        this.outtakeSubsystem = new Outtake(hardwareMap, this.telemetry, this.intakeSubsystem::forceDown);
        this.light = new IndicatorLight(hardwareMap, "light");

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(outtakeSubsystem);

        Constants.setConstants(FConstants.class, LConstants.class);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(Vector.cartesian(-2, 0).pose(0));

        this.limelight = new Limelight(hardwareMap, Limelight.Targets.BlueAndYellow);
        this.buffer = new Limelight.SampleState();
        this.cache = new Limelight.SampleState();
        this.makeDump = new PathMaker();
        this.makeDump.calculate(follower);

        limelight.raise();

        schedule(
                new RunCommand(follower::update),
                new RunCommand(telemetry::update),
                CMD.sampleAuto(
                        intakeSubsystem,
                        outtakeSubsystem,
                        follower,
                        light,
                        this::opModeIsActive,
                        limelight,
                        buffer,
                        cache,
                        telemetry,
                        makeDump
                )
        );
    }
}
