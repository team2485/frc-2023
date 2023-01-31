package frc.robot.commands.vision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.drive.Drivetrain;

public class AlignToTag extends CommandBase {
  private final PhotonCamera m_camera;
  private final Drivetrain m_drivetrain;
  private final Supplier<Pose2d> m_poseProvider;

  private final ProfiledPIDController m_XController = new ProfiledPIDController(1, .1, 0,
      Vision.kXConstraints);
  private final ProfiledPIDController m_YController = new ProfiledPIDController(1.1, .1, 0,
      Vision.kYConstraints);
  private final ProfiledPIDController m_OmegaController = new ProfiledPIDController(4.5, .3, .1,
      Vision.kOmegaConstraints);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  private boolean xAtGoalPos = false;
  private boolean yAtGoalPos = false;
  private boolean rotationAtGoalPos = false;

  ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

  public AlignToTag(PhotonCamera camera, Drivetrain drivetrain, Supplier<Pose2d> poseProvider) {
    this.m_camera = camera;
    this.m_drivetrain = drivetrain;
    this.m_poseProvider = poseProvider;

    m_XController.setTolerance(.1);
    m_YController.setTolerance(.1);
    m_OmegaController.setTolerance(Units.degreesToRadians(3));
    m_OmegaController.enableContinuousInput(-1, 1);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    var robotPose = m_poseProvider.get();

    goalPose = null;
    lastTarget = null;

    m_OmegaController.reset(robotPose.getRotation().getRadians());
    m_XController.reset(robotPose.getX());
    m_YController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = m_poseProvider.get();
    var result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      // find tag to chase
      var targetOpt = result.getTargets().stream().filter(t -> t.getFiducialId() == Vision.kTagOfInterest)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        if (!target.equals(lastTarget)) {
          lastTarget = target;
          var camToTarget = target.getBestCameraToTarget();
          var transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(),
              camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(0)));

          var cameraPose = robotPose.transformBy(
              new Transform2d(Vision.kCameraToRobot.getTranslation().toTranslation2d(), new Rotation2d())
                  .inverse());
          Pose2d targetPose = cameraPose.transformBy(transform);

          goalPose = targetPose.transformBy(Vision.kTagToGoal);
        }

        if (null != goalPose) {
          m_XController.setGoal(goalPose.getX());
          m_YController.setGoal(goalPose.getY());
          m_OmegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
    }

    var xSpeed = m_XController.calculate(robotPose.getX());

    if (Math.abs(goalPose.getX() - robotPose.getX()) < .15) {
      xSpeed = 0;
      xAtGoalPos = true;
    } else if (rotationAtGoalPos) {
      xAtGoalPos = false;
    }

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putBoolean("at goal", m_YController.atGoal());
    SmartDashboard.putNumber("goal pose", goalPose.getY());
    SmartDashboard.putNumber("robot pose", robotPose.getY());
    SmartDashboard.putBoolean("at Xgoal", xAtGoalPos);
    SmartDashboard.putBoolean("at Ygoal", yAtGoalPos);
    SmartDashboard.putBoolean("at Rptgoal", rotationAtGoalPos);

    var ySpeed = m_YController.calculate(robotPose.getY());

    if (Math.abs(goalPose.getY() - robotPose.getY()) < .25) {
      ySpeed = 0;
      yAtGoalPos = true;
    } else if (rotationAtGoalPos) {
      yAtGoalPos = false;
    }

    SmartDashboard.putNumber("ySpeed", ySpeed);
    // SmartDashboard.putNumber("rotDiff", goalPose.getRotation().getDegrees() -
    // robotPose.getRotation().getDegrees());

    var omegaSpeed = m_OmegaController.calculate(robotPose.getRotation().getRadians());

    if ((xAtGoalPos && yAtGoalPos)
        && (Math.abs(robotPose.getRotation().getDegrees() - goalPose.getRotation().getDegrees()) < 5))
      omegaSpeed = 0;
    else
      rotationAtGoalPos = false;

    SmartDashboard.putNumber("omegaSpeed", omegaSpeed);

    xSpeed = MathUtil.clamp(xSpeed, -3.0, 3.0);
    ySpeed = MathUtil.clamp(ySpeed, -3.0, 3.0);
    omegaSpeed = MathUtil.clamp(omegaSpeed, -3.0, 3.0);
    m_drivetrain.drive(new Translation2d(xSpeed, ySpeed), -omegaSpeed, true, true);
  }

  public void updateShuffleBoard() {
    var robotPose = m_poseProvider.get();
    SmartDashboard.putNumber("omegaSpeed", m_OmegaController.calculate(robotPose.getRotation().getRadians()));
  }
}
