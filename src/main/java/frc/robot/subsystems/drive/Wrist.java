package frc.robot.subsystems.drive;

import static frc.robot.Constants.*;
import static frc.robot.Constants.WristConstants.*;
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
    private final WL_SparkMax m_spark = new WL_SparkMax(kWristSparkPort);
    
    private final SR_ProfiledPIDController m_controller =
    new SR_ProfiledPIDController(kPWrist, kIWrist, kDWrist, kWristMotionProfileConstraints);


    @Log(name = "Wrist Feedforward")
    private final SR_ArmFeedforward m_feedforward =
        new SR_ArmFeedforward(
            kSWristVolts, kGWristVolts, kVWristVoltSecondsPerRadian, kAWristVoltSecondsSquaredPerRadian);
  

    @Log(name = "angle setpoint radians")
    private double m_angleSetpointRadiansCurrent = kWristBottomPositionRadians;
  
    private double m_angleSetpointRadiansFinal = m_angleSetpointRadiansCurrent;
  
    private double m_previousVelocitySetpoint = 0;
  
    private boolean m_isZeroed = false;


    public Wrist() {
        m_spark.enableVoltageCompensation(kNominalVoltage);
        m_spark.setSmartCurrentLimit(kWristSmartCurrentLimitAmps);
        m_spark.setSecondaryCurrentLimit(kWristImmediateCurrentLimitAmps);
    
        m_spark.setInverted(false);
    
        m_spark.setIdleMode(IdleMode.kBrake);
    
        m_controller.setTolerance(kWristControllerPositionTolerance);
    
        this.resetAngleRadians(kWristBottomPositionRadians);
    
        Shuffleboard.getTab("Wrist").add("Wrist controller", m_controller);
      }


    @Log(name = "Current angle (radians)")
    public double getAngleRadians() {
        return m_spark.getEncoder().getPosition() * kWristRadiansPerMotorRev;
    }

    @Config(name = "Set angle (radians)", defaultValueNumeric = kWristBottomPositionRadians)
    public void setAngleRadians(double angle) {
        m_angleSetpointRadiansCurrent =
            MathUtil.clamp(angle, kWristBottomPositionRadians, kWristTopPositionRadians);
    }

    public void resetAngleRadians(double angle) {
        m_spark.getEncoder().setPosition(angle / kWristRadiansPerMotorRev);
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
              kTimestepSeconds);
  
      m_previousVelocitySetpoint = m_controller.getSetpoint().velocity;
  
      m_spark.set((controllerVoltage + feedforwardVoltage) / kNominalVoltage);
    }
}
