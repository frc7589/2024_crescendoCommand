package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

public class DataContainer {
    public static Pose3d camPose = null;
    public static double camPoseTimestamp = 0;
    public static Pose2d pose2d = new Pose2d();

    public static final double getDistanceToSpeaker(double x, double y) {
        var alliance = DriverStation.getAlliance();
        
        if (x > 7.5) {
            return Math.sqrt(
                Math.pow(16.53-x, 2) + Math.pow(5.552-y, 2)
            );
        }

        return Math.sqrt(
            Math.pow(x, 2) + Math.pow(5.552-y, 2)
        );
    }
}
