package frc.util;

import java.io.IOException;
import java.lang.StackWalker.Option;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class PoseEstimation extends SubsystemBase {
  private final PhotonCamera m_camera;
  private final Drivetrain m_drivetrain;
  private final AprilTagFieldLayout aprilTagFieldLayout;

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final PhotonPoseEstimator m_photonPoseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  public PoseEstimation(PhotonCamera camera, Drivetrain drivetrain) {
    this.m_camera = camera;
    this.m_drivetrain = drivetrain;
    AprilTagFieldLayout layout;

    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      var alliance = DriverStation.getAlliance();
      layout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
          : OriginPosition.kRedAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }

    this.aprilTagFieldLayout = layout;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    m_poseEstimator = new SwerveDrivePoseEstimator(Swerve.swerveKinematics, m_drivetrain.getYaw(),
        m_drivetrain.getModulePositionsInverted(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);

    m_photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, camera, VisionConstants.kCameraToRobot.inverse());
    m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    tab.addString("Pose", this::getFormattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    // update pose estimator with visible targets
    // var result = m_camera.getLatestResult();
    // var resultTimestamp = result.getTimestampSeconds();
    // if (resultTimestamp != previousPipelineTimestamp && result.hasTargets()) {
    //   previousPipelineTimestamp = resultTimestamp;
    //   var target = result.getBestTarget();
    //   var fiducialId = target.getFiducialId();

    //   // get tab pose from field layout
    //   Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty()
    //       : aprilTagFieldLayout.getTagPose(fiducialId);
    //   if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
    //     var targetPose = tagPose.get();
    //     SmartDashboard.putNumber("tag id", fiducialId);
    //     SmartDashboard.putString("tag pose", "x: " + aprilTagFieldLayout.getTagPose(fiducialId).get().getX() + " y: " + aprilTagFieldLayout.getTagPose(fiducialId).get().getY());
    //     Transform3d camToTarget = target.getBestCameraToTarget();
    //     // TODO: check inverse
    //     Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

    //     var visionMeasurement = camPose.transformBy(VisionConstants.kCameraToRobot);

    //     m_poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
    //   }
    // }

    // m_poseEstimator.update(m_drivetrain.getYaw(), m_drivetrain.getModulePositionsInverted());

    // field2d.setRobotPose(getCurrentPose());

    m_poseEstimator.update(m_drivetrain.getYaw(), m_drivetrain.getModulePositionsInverted());

    Optional<EstimatedRobotPose> result = this.getEstimatedRobotPose(m_poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      m_poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      field2d.getObject("Cam Est Pose").setPose(camPose.estimatedPose.toPose2d());
    } else {
      field2d.getObject("Cam Est Pose").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }

    field2d.getObject("Actual Pos").setPose(m_poseEstimator.getEstimatedPosition());
    field2d.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d prevEstimatedRobotPose) {
    if (m_photonPoseEstimator == null) {
      return Optional.empty();
    }

    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimator.update();
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    m_poseEstimator.resetPosition(m_drivetrain.getYaw(), m_drivetrain.getModulePositionsInverted(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }
}
