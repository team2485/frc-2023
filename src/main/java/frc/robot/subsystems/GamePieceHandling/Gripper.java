// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GamePieceHandling;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.motorcontrol.base.WPI_SparkMax;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;
import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase implements Loggable{
  private final WPI_SparkMax m_spark = new WPI_SparkMax(kGripperSparkPort, MotorType.kBrushless);
  private final PIDController m_controller = new PIDController(3, 0.5, 0);

  @Log(name="setpoint")
  private double m_posSetpointMetersCurrent = 0;

  private double m_posSetpointMetersFinal = m_posSetpointMetersCurrent;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  @Log(name="piece")
  private String pieceType = "None";

  public enum m_pieceType {
    Cone,
    Cube,
    None,
  }

  private m_pieceType currentPieceType = m_pieceType.None;

  public enum m_gripperStates {
    StateFault,
    StateWait,
    StateInit,
    StateZero,
    StateGrip,
    StateIdle
  }

  public static m_gripperStates m_gripperState;
  public static m_gripperStates m_requestedState;

  private double stateTimer = 0;

  public Gripper() {
    m_gripperState = m_gripperStates.StateWait;

    m_spark.setSmartCurrentLimit(kGripperCurrentLimit);
    // some people were saying that this method was not properly implemented and does nothing
    // it probably works now as those complaints were from 2020 but check this if there are issues 
 
    m_controller.setTolerance(kGripperControllerPositionTolerance);

  }

  @Log(name = "output current")
  public double getVoltage(){
    return m_spark.getOutputCurrent();
  }

  @Log(name = "current pos")
  public double getGripperPosition() {
    return m_spark.getEncoder().getPosition() * kGripperRadiansPerMotorRev;
  }

  @Log(name = "pulse")
  public double getPulses(){
    return m_spark.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // only update the piece type when it would realistically change to prevent erratically swtiching between states as the encoder passes through them
    // This is from the current based approach
    //if (!isStalling()) return;

    // if (!isStopped()) return;

    // updateCurrentHeldPiece();

    switch (m_gripperState) {
        case StateFault:
            break;
        case StateWait:
          if(RobotState.isEnabled()){
            m_gripperState = m_gripperStates.StateInit;
          }
          break;
        case StateInit:
            stateTimer = 25;
            m_gripperState = m_gripperStates.StateZero;
            break;
        case StateZero:
            m_spark.setVoltage(-0.75);
            if (stateTimer == 0) {
                if (Math.abs(this.getEncoderVelocity()) < 0.01) {
                    this.resetEncoderPosition(0);
                    this.setPositionSetpoint(0);
                    m_spark.setVoltage(0);
                    m_gripperState = m_gripperStates.StateIdle;
                }
            } else {
                stateTimer--;
            }
            break;
        case StateGrip:
              this.setPositionSetpoint(1.55);
              m_gripperState = m_gripperStates.StateIdle;
            break;        
        case StateIdle:
            if (m_voltageOverride) {
                m_spark.set(m_voltageSetpoint / kNominalVoltage);
            } else {
                double controllerVoltage = m_controller.calculate(this.getGripperPosition(), m_posSetpointMetersCurrent);
                m_spark.set(controllerVoltage / kNominalVoltage);
                this.updateCurrentHeldPiece();
            }

            if (m_requestedState != null) m_gripperState = m_requestedState;
            m_requestedState = null;
            break;
    }
  }

  public void requestState(m_gripperStates state){
    m_requestedState = state;
  }

  private void updateCurrentHeldPiece() {
    //if are fully closed or open enough we don't have anything
    // if (Math.abs(getGripperPosition()) < kPieceDetectionTolerance 
    // || getGripperPosition() > kGripperOpenPositionSetpoint - kGripperControllerPositionTolerance)
    //   setCurrentHeldPiece(m_pieceType.None);
    // else if (Math.abs(getGripperPosition() - kCubeEncoderDistance) < GripperConstants.kPieceDetectionTolerance)
    //   setCurrentHeldPiece(m_pieceType.Cube);
    // else setCurrentHeldPiece(m_pieceType.Cone);

    if(Math.abs(this.getGripperPosition()-kCubeEncoderDistance)<=kPieceDetectionTolerance){
      setCurrentHeldPiece(m_pieceType.Cube);
    }else if(this.getGripperPosition()>kConeEncoderThreshold){
      setCurrentHeldPiece(m_pieceType.Cone);
    }else{
      setCurrentHeldPiece(m_pieceType.None);
    }
    pieceType = currentPieceType.name();
  }

  public void resetEncoderPosition(double pos) {
    m_spark.getEncoder().setPosition(pos);
  }


  @Log(name = "current vel")
  public double getEncoderVelocity() {
    return m_spark.getEncoder().getVelocity() * kGripperRadiansPerMotorRev;
  }

  @Log(name = "pos setpoint")
  public void setPositionSetpoint(double setpoint) {
    m_voltageOverride = false;
    m_posSetpointMetersCurrent = MathUtil.clamp(setpoint, kGripperOpenPosMeters, kGripperClosedPosMeters);
  }

  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public double getPIDControllerOutput() {
    return m_controller.calculate(getGripperPosition(), m_controller.getSetpoint());
  }

  public boolean isAtSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setCurrentHeldPiece(m_pieceType pieceType) {
    currentPieceType = pieceType;
  }

  public m_pieceType getCurrenPieceType() {
    return currentPieceType;
  }

  public boolean isStopped()
  {
    return Math.abs(getEncoderVelocity()) < kGripperStoppedVelocityTolerance;
  }

  public boolean isStalling() {
    // I don't know if the getOutputCurrent is given before or after the currentlimit is applied
    // Hopefully the >= works but if the gripper doesn't stop trying to close check here
    return m_spark.getOutputCurrent() >= GripperConstants.kGripperCurrentLimit;
  }
}