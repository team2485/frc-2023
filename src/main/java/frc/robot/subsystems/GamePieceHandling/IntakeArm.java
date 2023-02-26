// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GamePieceHandling;
import frc.robot.Constants.IntakeArmConstants;
import frc.WarlordsLib.sendableRichness.SR_ArmFeedforward;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstants.*;
import io.github.oblarg.oblog.annotations.Log;
import static frc.robot.Constants.IntakeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {
  /** Creates a new IntakeArm. */
  private WPI_TalonFX m_talonLeft = new WPI_TalonFX(IntakeArmConstants.kIntakeArmPortLeft);
  private WPI_TalonFX m_talonRight = new WPI_TalonFX(IntakeArmConstants.kIntakeArmPortRight);
  private double m_positionSetpointMeters;
  private final SR_ProfiledPIDController m_controller = 
    new SR_ProfiledPIDController(
      kPIntakeArm, 
      kIIntakeArm, 
      kDIntakeArm,
      kMotionProfileConstraints);

  
  public enum m_intakeArmStates {
    StateFault,
    StateWait,
    StateInit,
    StateZero,
    StateRetracted,
   // StateMiddle,
    StateDeployed,
    StateIdle
  }
  public static m_intakeArmStates m_intakeArmState;



  public IntakeArm() {
    m_intakeArmState = m_intakeArmStates.StateRetracted;
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    talonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
      true,
      kIntakeArmSupplyCurrentLimitAmps,
      kIntakeArmSupplyCurrentThresholdAmps,
      kIntakeArmSupplyCurrentThresholdTimeSecs);
  
    talonConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
      true,
      kIntakeArmSupplyCurrentLimitAmps,
      kIntakeArmSupplyCurrentThresholdAmps,
      kIntakeArmSupplyCurrentThresholdTimeSecs);

    m_talonLeft.configAllSettings(talonConfig);
    m_talonRight.configAllSettings(talonConfig);

    m_talonLeft.enableVoltageCompensation(true);
    m_talonLeft.setNeutralMode(NeutralMode.Brake);
    m_talonLeft.setInverted(true);
    m_talonLeft.configNeutralDeadband(0.001);

    m_talonRight.enableVoltageCompensation(true);
    m_talonRight.setNeutralMode(NeutralMode.Brake);
    m_talonRight.setInverted(false);
    m_talonRight.configNeutralDeadband(0.001);

    m_controller.setTolerance(0.05);

    this.resetPositionMeters(0);
  }

  public void setPositionMeters(double position) {
    
    m_positionSetpointMeters = MathUtil.clamp(position, 0, kMaxPositionMeters);
  }
  public double getPositionMeters() {
    return m_talonLeft.getSelectedSensorPosition() * kDistancePerPulse;
  }

  public double getError() {
    return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
  }
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
  public double getVelocityMetersPerSecond() {
    return m_talonLeft.getSelectedSensorVelocity() * kDistancePerPulse;
  }

  public void resetPositionMeters(double position){

    m_talonLeft.setSelectedSensorPosition(position / kDistancePerPulse);
    m_talonRight.setSelectedSensorPosition(position / kDistancePerPulse);
  }

  public void runControlLoop(){
    double feedbackOutputVoltage = 0;

    feedbackOutputVoltage = m_controller.calculate(this.getPositionMeters(), m_positionSetpointMeters);
    m_talonLeft.set(ControlMode.Position, feedbackOutputVoltage);
    m_talonRight.set(ControlMode.Position, feedbackOutputVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
