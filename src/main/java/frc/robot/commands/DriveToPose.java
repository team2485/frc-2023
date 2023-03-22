package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveToPose extends CommandBase {
  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private final Drivetrain m_drivetrain;
  private final Supplier<Pose2d> m_poseProvider;
  private boolean left;
  private boolean middle;
  private boolean isAuto;
  private final boolean useAllianceColor;

  private int invertForAuto;
  private int negateOmega;
  private double kDistanceFromTagToPole = kOffsetToNextScoringStation;
  private double fieldHeight;
  private double offset;
  private double zero;
  private double y;


  public DriveToPose(
      Drivetrain drivetrain,
      Supplier<Pose2d> poseProvider,
      boolean left,
      boolean middle,
      boolean useAllianceColor,
      boolean isAuto) {
    this(drivetrain, poseProvider, left, middle, isAuto, kDefaultXYContraints,
        kDefaultOmegaConstraints, useAllianceColor);
  }

  public DriveToPose(
      Drivetrain drivetrain,
      Supplier<Pose2d> poseProvider,
      boolean left,
      boolean middle,
      boolean isAuto,
      TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints,
      boolean useAllianceColor) {
    this.m_drivetrain = drivetrain;
    this.m_poseProvider = poseProvider;
    this.left = left;
    this.useAllianceColor = useAllianceColor;
    this.middle = middle;
    this.isAuto = isAuto;
  

    negateOmega = 1;
    invertForAuto = 1;

    m_xController = new ProfiledPIDController( X_kP,  X_kI,  X_kD,
        xyConstraints);
    m_yController = new ProfiledPIDController( Y_kP,  Y_kI,  Y_kD,
        xyConstraints);
    m_xController.setTolerance( kTranslationTolerance);
    m_yController.setTolerance( kTranslationTolerance);
    m_thetaController = new ProfiledPIDController( THETA_kP,  THETA_kI,
         THETA_kD, omegaConstraints);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_thetaController.setTolerance( kThetaTolerance);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize(){
    resetPIDControllers();
    var robotPose = m_poseProvider.get();
    zero = middle ? 0:1;
    offset = left?kDistanceFromTagToPole:-kDistanceFromTagToPole;

    if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      //eliminate offset if we go for middle (cube section)
      fieldHeight = 0;
      if(robotPose.getY()<4.4){
        y = kTopTagYPos + offset*zero;
      }else if(robotPose.getY()>4.4 && robotPose.getY()<6.07){
        y = kMiddleTagYPos + offset*zero;
      }else{
        y = kBottomTagYPos + offset*zero;
      }
    }else{
      fieldHeight = kFieldWidthMeters;  
      if(robotPose.getY()>fieldHeight - 4.4){
        y = kTopTagYPos - offset*zero;
      }else if(robotPose.getY()<fieldHeight - 4.4 && robotPose.getY()>fieldHeight - 6.07){
        y = kMiddleTagYPos - offset*zero;
      }else{
        y = kBottomTagYPos - offset*zero;
      }
    }

    m_thetaController.setGoal(Math.PI);
    if(isAuto){
      m_xController.setGoal(1.725);
    }else{
      m_xController.setGoal(1.8);
    }
    m_yController.setGoal(Math.abs(fieldHeight-y));
  }

  @Override
  public void execute() {
    if(RobotState.isAutonomous()){
      invertForAuto = -1;
    }else{
      invertForAuto = 1;
    }
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

    m_drivetrain.drive(new Translation2d(-xSpeed*invertForAuto, -ySpeed*invertForAuto), negateOmega*omegaSpeed, true, true);
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0,0), 0, true, true);
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
