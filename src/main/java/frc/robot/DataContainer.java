package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class DataContainer {
    public static Pose3d camPose = null;
    public static Pose2d pose2d = new Pose2d();

    public static final double getDistanceToSpeaker(double x, double y) {
        return Math.sqrt(
            Math.pow(x, 2) + Math.pow(5.552-y, 2)
        );
    }
}
