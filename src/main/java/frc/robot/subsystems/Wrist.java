package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.WristConstants.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.sendableRichness.SR_ArmFeedforward;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

public class Wrist extends SubsystemBase implements Loggable {
  private WPI_TalonFX m_talon = new WPI_TalonFX(kWristSparkPort);
  // private final WL_SparkMax m_spark = new WL_SparkMax(kWristSparkPort);

  private final SR_ProfiledPIDController m_controller = new SR_ProfiledPIDController(kPWrist, kIWrist, kDWrist,
      kWristMotionProfileConstraints);

  @Log(name = "Wrist Feedforward")
  private final SR_ArmFeedforward m_feedforward = new SR_ArmFeedforward(
      kSWristVolts, kGWristVolts, kVWristVoltSecondsPerRadian, kAWristVoltSecondsSquaredPerRadian);

  @Log(name = "angle setpoint radians")
  // sets starting angle as midpoint of wrist range
  private double m_angleSetpointRadiansCurrent = (kWristBottomPositionRadians + kWristTopPositionRadians) / 2;
  private double m_angleSetpointRadiansFinal = m_angleSetpointRadiansCurrent;
  private double m_previousVelocitySetpoint = 0;
  private boolean m_isZeroed = false;

  private enum m_wristStates {
    StateFault,
    StateInit,
    StateZero,
    StateRun,
    StateIdle
  }

  @Log(name = "current wrist state")
  private static m_wristStates m_wristState;

  public Wrist() {
    m_wristState = m_wristStates.StateInit;

    TalonFXConfiguration wristTalonConfig = new TalonFXConfiguration();
    wristTalonConfig.voltageCompSaturation = kNominalVoltage;
    wristTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    wristTalonConfig.velocityMeasurementWindow = 1;

    wristTalonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, kIndexerSupplyCurrentLimitAmps,
        kIndexerSupplyCurrentThresholdAmps, kIndexerSupplyCurrentThresholdTimeSecs);

    wristTalonConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, kIndexerStatorCurrentLimitAmps,
        kIndexerStatorCurrentThresholdAmps, kIndexerStatorCurrentThresholdTimeSecs);

    m_talon.configAllSettings(wristTalonConfig);
    m_talon.setNeutralMode(NeutralMode.Brake);
    m_talon.setInverted(false);
    m_talon.enableVoltageCompensation(true);

    m_controller.setTolerance(kWristControllerPositionTolerance);

    this.resetAngleRadians(kWristBottomPositionRadians);

    Shuffleboard.getTab("Wrist").add("Wrist controller", m_controller);
  }

  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_talon.getSelectedSensorPosition() * kWristRadiansPerMotorRev;
  }

  @Config(name = "Set angle (radians)", defaultValueNumeric = kWristBottomPositionRadians)
  public void setAngleRadians(double angle) {
    m_angleSetpointRadiansCurrent = MathUtil.clamp(angle, kWristBottomPositionRadians, kWristTopPositionRadians);
  }

  public void resetAngleRadians(double angle) {
    m_talon.setSelectedSensorPosition(angle / kWristRadiansPerMotorRev);
  }

  @Override
  public void periodic() {
    switch (m_wristState) {
      case StateFault:
        break;
      case StateInit:
        m_wristState = m_wristStates.StateZero;
        break;
      case StateZero:
        m_talon.set(-0.4);
        if (m_talon.getSelectedSensorVelocity() < 0.01) { // TODO: update with actual velocity
          this.resetAngleRadians(0);
          m_talon.set(0);
          m_wristState = m_wristStates.StateRun;
        }
        break;
      case StateRun:
        double controllerVoltage = m_controller.calculate(this.getAngleRadians(), m_angleSetpointRadiansCurrent);

        double feedforwardVoltage = m_feedforward.calculate(
            m_angleSetpointRadiansCurrent,
            m_previousVelocitySetpoint,
            m_controller.getSetpoint().velocity,
            kTimestepSeconds);

        m_previousVelocitySetpoint = m_controller.getSetpoint().velocity;

        m_talon.set((controllerVoltage + feedforwardVoltage) / kNominalVoltage);
        break;
      case StateIdle:
        break;
    }
  }
}
