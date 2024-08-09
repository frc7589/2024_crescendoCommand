package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DataContainer;
import frc.robot.RobotContainer;

public class PhotonSubsystem extends SubsystemBase {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera m_aprilTagCam;
    public static boolean reverse = false;
    
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
            double range;
            if (result.getMultiTagResult().estimatedPose.isPresent) {
                Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
                photonPoseEstimator.setReferencePose(new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation()));
                DataContainer.camPose = photonPoseEstimator.getReferencePose();

                if (
                    RobotState.isDisabled() || 
                    RobotContainer.getPose().getTranslation()
                        .getDistance(DataContainer.camPose.toPose2d().getTranslation()) < 0.5
                ) {
                    RobotContainer.addVisionMeasurement(DataContainer.camPose.toPose2d(), result.getTimestampSeconds());
                }
            }
            SmartDashboard.putBoolean("hasApriltag", result.hasTargets());
            if (result.hasTargets()) {
                range = result.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d(0,-Units.inchesToMeters(14),0));
                SmartDashboard.putNumber("speakerPosition", range);
                SmartDashboard.putNumber("April tag ID", result.getBestTarget().getFiducialId());
            }
        }

    }
}