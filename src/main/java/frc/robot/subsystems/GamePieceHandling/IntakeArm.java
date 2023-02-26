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

import frc.robot.subsystems.GamePieceHandling.Elevator.m_elevatorStates;
import io.github.oblarg.oblog.annotations.Log;
import static frc.robot.Constants.IntakeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {
  /** Creates a new IntakeArm. */
  private WPI_TalonFX m_talonLeft = new WPI_TalonFX(IntakeArmConstants.kIntakeArmPortLeft);
  private WPI_TalonFX m_talonRight = new WPI_TalonFX(IntakeArmConstants.kIntakeArmPortRight);
  private double m_positionSetpointAngle;
  public int stateTimer;
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

    this.resetAngle(0);
  }

  public void setPositionRadians(double position) {
    
    m_positionSetpointAngle= MathUtil.clamp(position, 0, kMaxPositionMeters);
  }
  public double getRadians() {
    return m_talonLeft.getSelectedSensorPosition() * kRadiansPerPulse;
  }

  public double getError() {
    return Math.abs(m_positionSetpointAngle - this.getRadians());
  }
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
  public double getRadiansPerSecond() {
    return m_talonLeft.getSelectedSensorVelocity() * kRadiansPerPulse;
  }

  public void resetAngle(double position){

    m_talonLeft.setSelectedSensorPosition(position / kRadiansPerPulse);
    m_talonRight.setSelectedSensorPosition(position / kRadiansPerPulse);
  }
  private boolean firstTime = true;
  double filteredAngle=0;
  public void runControlLoop(){
   
    if (firstTime) {
      filteredAngle = this.getRadians();
      firstTime = false;
    } else {
      filteredAngle += ((this.getRadians() - filteredAngle) * 0.5);
    }

    double feedbackOutputVoltage = 0;

    feedbackOutputVoltage = m_controller.calculate(filteredAngle, m_positionSetpointAngle)/Constants.kNominalVoltage;
    m_talonLeft.set(ControlMode.PercentOutput, feedbackOutputVoltage);
    m_talonRight.set(ControlMode.PercentOutput, feedbackOutputVoltage);

  }

  private m_intakeArmStates requestedState;
  public void requestState(m_intakeArmStates state){
    requestedState=state;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(m_intakeArmState){
      case StateFault: break;
      case StateWait: if(RobotState.isEnabled()){
        m_intakeArmState = m_intakeArmStates.StateInit;
        break;
      }
      case StateInit: stateTimer=25; m_intakeArmState=m_intakeArmStates.StateZero;
      case StateZero: m_talonLeft.setVoltage(-0.75); m_talonRight.setVoltage(-0.75);
        if(stateTimer==0){
          if (Math.abs(this.getRadiansPerSecond()) < 0.01) {
            this.resetAngle(0);
            this.setPositionRadians(0);
            m_talonLeft.setVoltage(0);
            m_talonRight.setVoltage(0);
            m_intakeArmState = m_intakeArmState.StateIdle;
          }

        }
        else{stateTimer--;} break;
      case StateRetracted:
        this.setPositionRadians(kIntakeArmRetractedPositionRadians);
        m_intakeArmState=m_intakeArmStates.StateIdle;
        break;
      case StateDeployed:
        this.setPositionRadians(kIntakeArmDeployedPositionRadian);
        m_intakeArmState=m_intakeArmStates.StateIdle;
        break;
      case StateIdle:
        this.runControlLoop();
        if(requestedState==null){break;}
        m_intakeArmState=requestedState;
        requestedState=null;
        break;
    }
  }
}
