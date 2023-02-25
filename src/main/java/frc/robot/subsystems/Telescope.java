// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TelescopeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.WarlordsLib.motorcontrol.base.WPI_SparkMax;
import frc.WarlordsLib.sendableRichness.SR_ElevatorFeedforward;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import frc.WarlordsLib.sendableRichness.SR_TrapezoidProfile;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  // private WPI_TalonFX m_spark = new WL_TalonFX(kTelescopePort);
  private WPI_SparkMax m_spark = new WPI_SparkMax(kTelescopePort, MotorType.kBrushless);

  private static final double kTelescopeStartPosition = 0;

  private double m_feedForwardVoltage, m_positionSetpointMeters, m_voltageSetpoint, m_lastVelocitySetpoint;
  private double m_outputPercentage, m_feedbackOutput, m_feedforwardOutput;
  private boolean m_isEnabled, m_voltageOverride;

  private final SR_ElevatorFeedforward m_feedforward = new SR_ElevatorFeedforward(
      kSTelescopeVolts, kGTelescopeVolts, kVTelescopeVoltSecondsPerMeter, kATelescopeVoltSecondsSquaredPerMeter);

  SR_ProfiledPIDController telescopeController = new SR_ProfiledPIDController(
      kPTelescope,
      kITelescope,
      kDTelescope, new SR_TrapezoidProfile.Constraints(2, 1));

  public Telescope() {
    m_feedForwardVoltage = 0;
    m_positionSetpointMeters = 0;
    m_voltageSetpoint = 0;
    m_lastVelocitySetpoint = 0;

    m_outputPercentage = 0;
    m_feedbackOutput = 0;
    m_feedforwardOutput = 0;

    m_isEnabled = false;
    m_voltageOverride = false;

    m_spark.setSmartCurrentLimit(kTelescopeSmartCurrentLimitAmps);
    m_spark.setInverted(false);
    m_spark.setIdleMode(IdleMode.kBrake);

    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);

    this.resetPositionMeters(0);

    
  }

  public void setPositionMeters(double position) {
    m_voltageOverride = false;
    m_positionSetpointMeters = MathUtil.clamp(position, kTelescopeStartPosition, kTelescopeMaxPosition);
  }

  public double getError() {
    return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
  }

  public double getPositionMeters() {
    return m_spark.getEncoder().getPosition();
  }

  public void resetPositionMeters(double position) {
    m_spark.getEncoder().setPosition(position);
  }

  public double getVelocityMetersPerSecond() {
    return m_spark.getEncoder().getVelocity();
  }

  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;

  }

  public void runControlLoop(){
    if (m_voltageOverride) {
      m_spark.set(m_voltageSetpoint / Constants.kNominalVoltage);
    } else {
      double feedbackOutputVoltage = 0;

      feedbackOutputVoltage = telescopeController.calculate(this.getPositionMeters(), m_positionSetpointMeters);

      double feedForwardOutputVoltage = 0;

      feedForwardOutputVoltage = m_feedforward.calculate(m_lastVelocitySetpoint,
          telescopeController.getSetpoint().velocity, kTelescopeControlLoopTimeSeconds);

      m_outputPercentage = (feedbackOutputVoltage + feedForwardOutputVoltage) / Constants.kNominalVoltage; // same deal as above

      m_feedbackOutput = feedbackOutputVoltage;
      m_feedforwardOutput = feedForwardOutputVoltage;

      m_spark.set(m_outputPercentage);

      m_lastVelocitySetpoint = telescopeController.getSetpoint().velocity;
    }
  }

  @Override
  public void periodic() {

    
  }
}
