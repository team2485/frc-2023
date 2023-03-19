// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GamePieceHandling;

import frc.robot.Constants.IntakeArmConstants;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.robot.Constants;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import static frc.robot.Constants.IntakeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase implements Loggable {
  /** Creates a new IntakeArm. */
  private WPI_TalonFX m_talonLeft = new WPI_TalonFX(IntakeArmConstants.kIntakeArmPortLeft);
  private WPI_TalonFX m_talonRight = new WPI_TalonFX(IntakeArmConstants.kIntakeArmPortRight);

  @Log(name = "setpoint")
  private double m_positionSetpointAngle;

  @Log(name = "stateTimer")
  public int stateTimer = 0;
  public int stateTimer2 = 0;

  private final SR_ProfiledPIDController m_controller = new SR_ProfiledPIDController(
      kPIntakeArm,
      kIIntakeArm,
      kDIntakeArm,
      kMotionProfileConstraints);

  public static enum m_intakeArmStates {
    StateFault,
    StateWait,
    StateInit,
    StateZero,
    StateRetracted,
    // StateMiddle,
    StateDeployed,
    StateDeployAndLock,
    StateIdle
  }

  public static m_intakeArmStates m_intakeArmState;

  public IntakeArm() {
    m_intakeArmState = m_intakeArmStates.StateWait;
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    talonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        true,
        kIntakeArmSupplyCurrentLimitAmps,
        kIntakeArmSupplyCurrentThresholdAmps,
        kIntakeArmSupplyCurrentThresholdTimeSecs);

    talonConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        true,
        kIntakeArmStatorCurrentLimitAmps,
        kIntakeArmStatorCurrentThresholdAmps,
        kIntakeArmStatorCurrentThresholdTimeSecs);

    m_talonLeft.configAllSettings(talonConfig);
    m_talonRight.configAllSettings(talonConfig);

    // m_talonLeft.enableVoltageCompensation(true);
    m_talonLeft.setNeutralMode(NeutralMode.Brake);
    m_talonLeft.setInverted(false);
    m_talonLeft.configNeutralDeadband(0.001);

    // m_talonRight.enableVoltageCompensation(true);
    m_talonRight.setNeutralMode(NeutralMode.Brake);
    m_talonRight.setInverted(true);
    m_talonRight.configNeutralDeadband(0.001);

    m_controller.setTolerance(0.05);

    this.resetAngle(0);
  }

  public void setPositionRadians(double position) {

    m_positionSetpointAngle = MathUtil.clamp(position, 0, kMaxPositionMeters);
  }

  @Log(name = "radians")
  public double getRadians() {
    return m_talonLeft.getSelectedSensorPosition() * kRadiansPerPulse;
  }

  public double getError() {
    return Math.abs(m_positionSetpointAngle - this.getRadians());
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Log(name = "radians per sec")
  public double getRadiansPerSecond() {
    return m_talonLeft.getSelectedSensorVelocity() * kRadiansPerPulse;
  }

  @Log(name = "output current")
  public double getCurrent() {
    return m_talonLeft.getStatorCurrent();
  }

  public void resetAngle(double position) {

    m_talonLeft.setSelectedSensorPosition(position / kRadiansPerPulse);
    m_talonRight.setSelectedSensorPosition(position / kRadiansPerPulse);
  }

  private boolean firstTime = true;
  double filteredAngle = 0;

  public void runControlLoop() {

    if (firstTime) {
      filteredAngle = this.getRadians();
      firstTime = false;
    } else {
      filteredAngle += ((this.getRadians() - filteredAngle) * 0.5);
    }

    double feedbackOutputVoltage = 0;

    feedbackOutputVoltage = m_controller.calculate(filteredAngle, m_positionSetpointAngle) / Constants.kNominalVoltage;
    m_talonLeft.set(ControlMode.PercentOutput, feedbackOutputVoltage);
    m_talonRight.set(ControlMode.PercentOutput, feedbackOutputVoltage);
  }

  private m_intakeArmStates requestedState;

  public void requestState(m_intakeArmStates state) {
    requestedState = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_intakeArmState) {
      case StateFault:
        break;
      case StateWait:
        if (RobotState.isEnabled()) {
          m_intakeArmState = m_intakeArmStates.StateInit;
        }
        break;
      case StateInit:
        stateTimer = 50;
        m_intakeArmState = m_intakeArmStates.StateZero;
        break;
      case StateZero:
        m_talonLeft.setVoltage(-0.75);
        m_talonRight.setVoltage(-0.75);
        if (stateTimer == 0) {
          if (this.getCurrent() > 20) {
            this.resetAngle(0);
            this.setPositionRadians(0);
            m_talonLeft.setVoltage(0);
            m_talonRight.setVoltage(0);
            m_intakeArmState = m_intakeArmStates.StateIdle;
          }
        } else {
          stateTimer--;
        }
        break;
      case StateRetracted:
        IntakeServo.release();
        if (stateTimer2 == 0) {
          this.setPositionRadians(kIntakeArmRetractedPositionRadians);
        } else {
          stateTimer2--;
        }
        this.runControlLoop();
        if (requestedState != null) {
          m_intakeArmState = requestedState;
          requestedState = null;
        }
        break;
      case StateDeployed:
        this.setPositionRadians(kIntakeArmDeployedPositionRadians);
        this.runControlLoop();
        if (requestedState != null) {
          m_intakeArmState = requestedState;
          requestedState = null;
        }
        break;
      case StateDeployAndLock:
        this.setPositionRadians(kIntakeArmDeployedPositionRadians);
        if (this.getRadians() > 1.2) {
          IntakeServo.lock();
        }
        this.runControlLoop();
        if (requestedState != null) {
          stateTimer2 = 25;
          m_intakeArmState = requestedState;
          requestedState = null;
        }
        break;
      case StateIdle:
        this.runControlLoop();
        if (requestedState != null) {
          m_intakeArmState = requestedState;
          requestedState = null;
        }
        break;
    }
  }
}
