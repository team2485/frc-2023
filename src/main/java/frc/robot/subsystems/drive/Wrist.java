package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.WarlordsLib.sendableRichness.SR_ArmFeedforward;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;


public class Wrist extends SubsystemBase implements Loggable{
    private final WL_SparkMax m_spark = new WL_SparkMax(Constants.WristConstants.kWristSparkPort);
    
    private final SR_ProfiledPIDController m_controller =
    new SR_ProfiledPIDController(Constants.WristConstants.kPWrist, Constants.WristConstants.kIWrist, Constants.WristConstants.kDWrist, Constants.WristConstants.kWristMotionProfileConstraints);


    @Log(name = "Wrist Feedforward")
    private final SR_ArmFeedforward m_feedforward =
        new SR_ArmFeedforward(
            Constants.WristConstants.kSWristVolts, Constants.WristConstants.kGWristVolts, Constants.WristConstants.kVWristVoltSecondsPerRadian, Constants.WristConstants.kAWristVoltSecondsSquaredPerRadian);
  

    @Log(name = "angle setpoint radians")
    private double m_angleSetpointRadiansCurrent = Constants.WristConstants.kWristBottomPositionRadians;
  
    private double m_angleSetpointRadiansFinal = m_angleSetpointRadiansCurrent;
  
    private double m_previousVelocitySetpoint = 0;
  
    private boolean m_isZeroed = false;


    public Wrist() {
        m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
        m_spark.setSmartCurrentLimit(Constants.WristConstants.kWristSmartCurrentLimitAmps);
        m_spark.setSecondaryCurrentLimit(Constants.WristConstants.kWristImmediateCurrentLimitAmps);
    
        m_spark.setInverted(false);
    
        m_spark.setIdleMode(IdleMode.kBrake);
    
        m_controller.setTolerance(Constants.WristConstants.kWristControllerPositionTolerance);
    
        this.resetAngleRadians(Constants.WristConstants.kWristBottomPositionRadians);
    
        Shuffleboard.getTab("Wrist").add("Wrist controller", m_controller);
      }


    @Log(name = "Current angle (radians)")
    public double getAngleRadians() {
        return m_spark.getEncoder().getPosition() * Constants.WristConstants.kWristRadiansPerMotorRev;
    }

    @Config(name = "Set angle (radians)", defaultValueNumeric = Constants.WristConstants.kWristBottomPositionRadians)
    public void setAngleRadians(double angle) {
        m_angleSetpointRadiansCurrent =
            MathUtil.clamp(angle, Constants.WristConstants.kWristBottomPositionRadians, Constants.WristConstants.kWristTopPositionRadians);
    }

    public void resetAngleRadians(double angle) {
        m_spark.getEncoder().setPosition(angle / Constants.WristConstants.kWristRadiansPerMotorRev);
    }


    @Override
    public void periodic() {
  
      double controllerVoltage =
          m_controller.calculate(this.getAngleRadians(), m_angleSetpointRadiansCurrent);
  
      double feedforwardVoltage =
          m_feedforward.calculate(
              m_angleSetpointRadiansCurrent,
              m_previousVelocitySetpoint,
              m_controller.getSetpoint().velocity,
              Constants.kTimestepSeconds);
  
      m_previousVelocitySetpoint = m_controller.getSetpoint().velocity;
  
      m_spark.set((controllerVoltage + feedforwardVoltage) / Constants.kNominalVoltage);
    }
}
