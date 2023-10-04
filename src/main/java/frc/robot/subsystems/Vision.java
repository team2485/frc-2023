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
    // PhotonVision class that takes vision measurements from camera and triangulates position on field
    private final PhotonPoseEstimator m_photonPoseEstimator;
    // creates new PhotonCamera object for camera
    private final PhotonCamera m_camera;
    // creates a thread-safe object reference since mutliple robot poses could be reported concurrently and conflict
    // lowkey an interesting read, the atomic library gaslights machine instruction algos like compare-and-swap
    // that are instrinsically atomic into working for concurrency
    private final AtomicReference<EstimatedRobotPose> m_atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    public Vision() {
        this.m_camera = new PhotonCamera(VisionConstants.kCameraName); // initialize with a USEFUL name
        PhotonPoseEstimator photonPoseEstimator = null;

        try {
            var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(); // attempt to load the AprilTagFieldLayout
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

            if (m_camera != null) {
                photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, m_camera,
                        VisionConstants.kRobotToCamera); // MULTI_TAG_PNP uses all cameras in view for positioning
            }
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace()); // can't estimate poses without known tag positions
            photonPoseEstimator = null;
        }

        this.m_photonPoseEstimator = photonPoseEstimator;
    }

    @Override
    public void run() {
        if (m_photonPoseEstimator != null && m_camera != null) { // environment and camera must be initialized properly
            var photonResults = m_camera.getLatestResult(); // continuously get latest camera reading

            if (photonResults.hasTargets()
                    && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < 0.2)) { // need accurate readings
                m_photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                    var estimatedPose = estimatedRobotPose.estimatedPose;

                    if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= VisionConstants.kFieldLengthMeters
                            && estimatedPose.getY() > 0.0
                            && estimatedPose.getY() <= VisionConstants.kFieldWidthMeters) { // idiot check
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
