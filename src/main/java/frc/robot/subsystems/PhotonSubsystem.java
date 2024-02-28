package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonSubsystem extends SubsystemBase {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera m_aprilTagCam;

    public PhotonSubsystem() {
        this.m_aprilTagCam = new PhotonCamera("AprilTagCam");

        PhotonPoseEstimator photonPoseEstimator = null;

        var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        if (m_aprilTagCam != null) {
            photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, m_aprilTagCam, Constants.kRobotToCam);
        }
        this.photonPoseEstimator = photonPoseEstimator;
    }

    @Override
    public void periodic() {
        if (photonPoseEstimator != null && m_aprilTagCam != null) {
            var result = m_aprilTagCam.getLatestResult();
            if (result.getMultiTagResult().estimatedPose.isPresent) {
                Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
                photonPoseEstimator.setReferencePose(new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation()));
            }
        }

        SmartDashboard.putNumberArray("pose", new double[] {
            getPose3d().toPose2d().getX(),
            getPose3d().toPose2d().getY(),
            getPose3d().toPose2d().getRotation().getDegrees()
        });
    }

    public Pose3d getPose3d() {
        return photonPoseEstimator.getReferencePose() != null ?photonPoseEstimator.getReferencePose() : new Pose3d();
    }

}