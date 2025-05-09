package core.math;

import com.pedropathing.follower.Follower;
public class RobotPosition {
    public Vector position;
    public double rotation;

    public RobotPosition findRelativeToCurrent(Follower follower, Vector relative) {
        RobotPosition current = new RobotPosition(
                Vector.cartesian(
                        follower.getPose().getX(),
                        follower.getPose().getY()
                ),
                follower.getPose().getHeading()
        );

        return relativePosition(current, relative);
    }

    public RobotPosition(Vector position, double rotation) {
        this.position = position;
        this.rotation = rotation;
    }

    public static RobotPosition relativePosition(RobotPosition pos, Vector relative) {
        Vector relativePosFieldCentric = Vector.cartesian(
                relative.y * Math.cos(pos.rotation) + relative.x * Math.cos(pos.rotation - Math.toRadians(90)),
                relative.y * Math.sin(pos.rotation) + relative.x * Math.sin(pos.rotation - Math.toRadians(90))
        );

        Vector targetPosFieldCentric = relativePosFieldCentric.add(pos.position);
        return new RobotPosition(targetPosFieldCentric, pos.rotation);
    }
}
