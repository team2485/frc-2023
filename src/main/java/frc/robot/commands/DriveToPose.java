package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveToPose extends CommandBase {
  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private final Drivetrain m_drivetrain;
  private final Supplier<Pose2d> m_poseProvider;
  private final Pose2d goalPose;
  private final boolean useAllianceColor;
  private int negateOmega;

  public DriveToPose(
      Drivetrain drivetrain,
      Supplier<Pose2d> poseProvider,
      Pose2d goalPose,
      boolean useAllianceColor) {
    this(drivetrain, poseProvider, goalPose, VisionConstants.kDefaultXYContraints,
        VisionConstants.kDefaultOmegaConstraints, useAllianceColor);
  }

  public DriveToPose(
      Drivetrain drivetrain,
      Supplier<Pose2d> poseProvider,
      Pose2d goalPose,
      TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints,
      boolean useAllianceColor) {
    this.m_drivetrain = drivetrain;
    this.m_poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.useAllianceColor = useAllianceColor;
    negateOmega = 1;

    m_xController = new ProfiledPIDController(VisionConstants.X_kP, VisionConstants.X_kI, VisionConstants.X_kD,
        xyConstraints);
    m_yController = new ProfiledPIDController(VisionConstants.Y_kP, VisionConstants.Y_kI, VisionConstants.Y_kD,
        xyConstraints);
    m_xController.setTolerance(VisionConstants.kTranslationTolerance);
    m_yController.setTolerance(VisionConstants.kTranslationTolerance);
    m_thetaController = new ProfiledPIDController(VisionConstants.THETA_kP, VisionConstants.THETA_kI,
        VisionConstants.THETA_kD, omegaConstraints);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_thetaController.setTolerance(VisionConstants.kThetaTolerance);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;

    if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Translation2d transformedTranslation = new Translation2d(pose.getX(),
        pose.getY());
      Rotation2d transformedHeading = pose.getRotation();
      pose = new Pose2d(transformedTranslation, transformedHeading);
    }

    m_thetaController.setGoal(Math.abs(pose.getRotation().getRadians()));
    m_xController.setGoal(pose.getX());
    m_yController.setGoal(pose.getY());
  }

  @Override
  public void execute() {
    var robotPose = m_poseProvider.get();

    var xSpeed = m_xController.calculate(robotPose.getX());
    if (m_xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = m_yController.calculate(robotPose.getY());
    if (m_yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = m_thetaController.calculate(Math.abs(robotPose.getRotation().getRadians()));

    if(robotPose.getRotation().getRadians()<0){
        negateOmega = 1;
    }else{
        negateOmega = -1;
    }
    
    if (m_thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    m_drivetrain.drive(new Translation2d(-xSpeed, -ySpeed), -omegaSpeed, true, true);
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0,0), 0, false, false);
  }

  public boolean atGoal() {
    return m_xController.atGoal() && m_yController.atGoal() && m_thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = m_poseProvider.get();
    m_thetaController.reset(robotPose.getRotation().getRadians());
    m_xController.reset(robotPose.getX());
    m_yController.reset(robotPose.getY());
  }
}
