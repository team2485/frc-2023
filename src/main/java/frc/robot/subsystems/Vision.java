package frc.robot.subsystems;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.VisionConstants;

public class Vision implements Runnable {
    private final PhotonPoseEstimator m_photonPoseEstimator;
    private final PhotonCamera m_camera;
    private final AtomicReference<EstimatedRobotPose> m_atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    public Vision() {
        this.m_camera = new PhotonCamera(VisionConstants.kCameraName);
        PhotonPoseEstimator photonPoseEstimator = null;

        try {
            var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

            if (m_camera != null) {
                photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, m_camera, VisionConstants.kRobotToCamera);
            }
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }

        this.m_photonPoseEstimator = photonPoseEstimator;
    }

    @Override
    public void run() {
        if (m_photonPoseEstimator != null && m_camera != null && !RobotState.isAutonomous()) {
            var photonResults = m_camera.getLatestResult();

            if (photonResults.hasTargets() 
                && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < 0.2)) {
                m_photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                    var estimatedPose = estimatedRobotPose.estimatedPose;

                    if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= VisionConstants.kFieldLengthMeters
                        && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= VisionConstants.kFieldWidthMeters) {
                        m_atomicEstimatedRobotPose.set(estimatedRobotPose);
                    }
                });
            }
        }
    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return m_atomicEstimatedRobotPose.getAndSet(null);
    }
}
