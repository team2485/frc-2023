// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TelescopeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  private WPI_TalonFX m_telescope = new WL_TalonFX(kTelescopePort);

  private static final double kTelescopeStartPosition = 0;

  private double m_feedForwardVoltage, m_positionSetpointMeters, m_voltageSetpoint, m_lastVelocitySetpoint;
  private double m_outputPercentage, m_feedbackOutput, m_feedforwardOutput;
  private boolean m_isEnabled, m_voltageOverride;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
      kSTelescopeVolts, kVTelescopeVoltSecondsPerMeter, kATelescopeVoltSecondsSquaredPerMeter);

  ProfiledPIDController telescopeController = new ProfiledPIDController(
      kPTelescope,
      kITelescope,
      kDTelescope, new TrapezoidProfile.Constraints(5, 10));

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
  }

  public boolean isInverted() {
    return m_telescope.getInverted();
  }

  public void invertTalon() {
    if (m_telescope.getInverted())
      m_telescope.setInverted(false);
    else
      m_telescope.setInverted(true);
  }

  public void setLowerNodePositionMeters(double position) {
    m_voltageOverride = false;
    m_positionSetpointMeters = MathUtil.clamp(position, kTelescopeStartPosition, kTelescopeLowerPosition);
  }

  public void setMiddleNodePositionMeters(double position) {
    m_voltageOverride = false;
    m_positionSetpointMeters = MathUtil.clamp(position, kTelescopeStartPosition, kTelescopeMiddlePosition);
  }

  public void setUpperNodePositionMeters(double position) {
    m_voltageOverride = false;
    m_positionSetpointMeters = MathUtil.clamp(position, kTelescopeStartPosition, kTelescopeUpperPosition);
  }

  public double getError() {
    return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
  }

  public double getPositionMeters() {
    return m_telescope.getSelectedSensorPosition();
  }

  public void resetPositionMeters(double position) {
    m_telescope.setSelectedSensorPosition(position);
  }

  public double getVelocityMetersPerSecond() {
    return m_telescope.getSelectedSensorVelocity();
  }

  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_voltageOverride) {
      m_telescope.set(ControlMode.PercentOutput, m_voltageSetpoint / Constants.kNominalVoltage);
    } else {
      double feedbackOutputVoltage = 0;

      feedbackOutputVoltage = telescopeController.calculate(this.getPositionMeters(), m_positionSetpointMeters);

      double feedForwardOutputVoltage = 0;

      feedForwardOutputVoltage = m_feedforward.calculate(m_lastVelocitySetpoint,
          telescopeController.getSetpoint().velocity, kTelescopeControlLoopTimeSeconds);

      m_outputPercentage = (feedbackOutputVoltage + feedForwardOutputVoltage / 12); // same deal as above

      m_feedbackOutput = feedbackOutputVoltage;
      m_feedforwardOutput = feedForwardOutputVoltage;

      m_telescope.set(ControlMode.PercentOutput, m_outputPercentage);

      m_lastVelocitySetpoint = telescopeController.getSetpoint().velocity;
    }
  }
}
