package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class Vision extends SubsystemBase {
  private final PhotonCamera m_camera = new PhotonCamera(VisionConstants.kCameraName);
  private final Drivetrain m_drivetrain;
  private final Supplier<Pose2d> m_poseProvider;

  private final ProfiledPIDController m_XController = new ProfiledPIDController(
      (1.35 / 1.65) * ((1 + (0.18 * (.2)) / (1 - .2))), (2.5 - 2 * .75) / (1 - (.39 * .2)),
      (.37 - (.37 * .1)) / (1 - (.81 * .2)),
      VisionConstants.kXConstraints);
  private final ProfiledPIDController m_YController = new ProfiledPIDController(
      (1.35 / 1.65) * ((1 + (0.18 * (.2)) / (1 - .2))), (2.5 - 2 * .75) / (1 - (.39 * .2)),
      (.37 - (.37 * .1)) / (1 - (.81 * .2)),
      VisionConstants.kYConstraints);
  private final ProfiledPIDController m_OmegaController = new ProfiledPIDController(6, .2, .3,
      VisionConstants.kOmegaConstraints);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  private boolean xAtGoalPos = false;
  private boolean yAtGoalPos = false;
  private boolean rotationAtGoalPos = false;

  Alliance alliance;

  ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

  private double offset = 0;

  private Optional<PhotonTrackedTarget> targetOpt;

  int[] redOmittedTags = new int[] { 5, 6, 7, 8 };
  int[] blueOmittedTags = new int[] { 1, 2, 3, 4 };

  public enum m_visionStates {
    StateFault,
    StateWait,
    StateInit,
    StateAlign,
    StateIdle
  }

  public static m_visionStates m_visionState;
  public static m_visionStates m_requestedState;

  public Vision(Drivetrain drivetrain, Supplier<Pose2d> poseProvider) {
    this.m_drivetrain = drivetrain;
    this.m_poseProvider = poseProvider;

    m_XController.setTolerance(.1);
    m_YController.setTolerance(.1);
    m_OmegaController.setTolerance(Units.degreesToRadians(3));
    m_OmegaController.enableContinuousInput(-1, 1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("curr vision state", m_visionState.name());

    switch (m_visionState) {
      case StateFault:
        break;
      case StateWait:
        if (RobotState.isEnabled())
          m_visionState = m_visionStates.StateInit;
        break;
      case StateInit:
        offset = 0;
        var robotPose = m_poseProvider.get();

        goalPose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        lastTarget = null;

        alliance = DriverStation.getAlliance();

        m_OmegaController.reset(robotPose.getRotation().getRadians());
        m_XController.reset(robotPose.getX());
        m_YController.reset(robotPose.getY());
        m_visionState = m_visionStates.StateIdle;
        break;
      case StateAlign:
        robotPose = m_poseProvider.get();
        var result = m_camera.getLatestResult();

        // make sure that we are not detecting tags on the other side of the field
        if (alliance == Alliance.Blue && result.getBestTarget().getFiducialId() > 4
            || alliance == Alliance.Red && result.getBestTarget().getFiducialId() < 5)
          m_visionState = m_visionStates.StateFault; // TODO: actually handle error
        if (result.hasTargets()) {
          // find tag to chase
          targetOpt = result.getTargets().stream()
              .filter(t -> t.getFiducialId() == result.getBestTarget().getFiducialId())
              .findFirst();
          if (targetOpt.isPresent()) {
            var target = targetOpt.get();
            if (!target.equals(lastTarget)) {
              lastTarget = target;
              var camToTarget = target.getBestCameraToTarget();
              var transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(),
                  camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(0)));

              var cameraPose = robotPose.transformBy(
                  new Transform2d(VisionConstants.kCameraToRobot.getTranslation().toTranslation2d(), new Rotation2d())
                      .inverse());
              Pose2d targetPose = cameraPose.transformBy(transform);

              goalPose = targetPose.transformBy(new Transform2d(new Translation2d(1, offset),
                  Rotation2d.fromDegrees(180)));
            }

            if (null != goalPose) {
              m_XController.setGoal(goalPose.getX());
              m_YController.setGoal(goalPose.getY());
              m_OmegaController.setGoal(goalPose.getRotation().getRadians());
            }
          }
        }

        // omega is the rotation relative to the April Tag
        var omegaSpeed = m_OmegaController.calculate(robotPose.getRotation().getRadians());

        // run rotation last so it doesn't mess up the translation
        if ((xAtGoalPos && yAtGoalPos)
            && (Math.abs(robotPose.getRotation().getDegrees() - goalPose.getRotation().getDegrees()) < 5)) {
          omegaSpeed = 0;
          rotationAtGoalPos = true;
        } else
          rotationAtGoalPos = false;

        // x is forward and backward relative to the April Tag
        var xSpeed = m_XController.calculate(robotPose.getX());

        if (Math.abs(goalPose.getX() - robotPose.getX()) < .15) {
          xSpeed = 0;
          xAtGoalPos = true;
        }
        // checking if x is out of the tolerance but waiting until rotation is completed
        // to fix it
        // this works because rotation is run first and execute is called before
        // isFinished in the command sequence
        else if (rotationAtGoalPos) {
          xAtGoalPos = false;
        }

        // y is the horizontal direction relative to the April Tag
        var ySpeed = m_YController.calculate(robotPose.getY());

        if (Math.abs(goalPose.getY() - robotPose.getY()) < .25) {
          ySpeed = 0;
          yAtGoalPos = true;
        }
        // checking if y is out of the tolerance but waiting until rotation is completed
        // to fix it
        // this works because rotation is run first and execute is called before
        // isFinished in the command sequence
        else if (rotationAtGoalPos) {
          yAtGoalPos = false;
        }

        m_drivetrain.drive(new Translation2d(xSpeed, ySpeed), -omegaSpeed, true, true);

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putBoolean("at goal", m_YController.atGoal());
        SmartDashboard.putNumber("goal pose", goalPose.getY());
        SmartDashboard.putNumber("robot pose", robotPose.getY());
        SmartDashboard.putBoolean("at Xgoal", xAtGoalPos);
        SmartDashboard.putBoolean("at Ygoal", yAtGoalPos);
        SmartDashboard.putBoolean("at Rptgoal", rotationAtGoalPos);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        // SmartDashboard.putNumber("rotDiff", goalPose.getRotation().getDegrees() -
        // robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("omegaSpeed", omegaSpeed);
        SmartDashboard.putNumber("pose rotations", robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("difference between degree headings",
            Math.abs(robotPose.getRotation().getDegrees() - goalPose.getRotation().getDegrees()));
        break;
      case StateIdle:
        break;
    }
  }

  public void requestState(m_visionStates state) {
    m_requestedState = state;
  }

  public void updateShuffleBoard() {
    var robotPose = m_poseProvider.get();
    SmartDashboard.putNumber("omegaSpeed", m_OmegaController.calculate(robotPose.getRotation().getRadians()));
  }

  public void addOffset(double newOffset) {
    offset += newOffset;
  }

  public void zeroOffset() {
    offset = 0;
  }
}
