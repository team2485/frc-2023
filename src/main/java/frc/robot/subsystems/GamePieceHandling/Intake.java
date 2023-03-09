// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GamePieceHandling;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.WarlordsLib.motorcontrol.base.WPI_SparkMax;
import frc.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import com.revrobotics.CANSparkMax.IdleMode;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  public static enum m_intakeStates {
    StateFault,
    StateWait,
    StateInit,
    StateOn,
    StateOff,
    StateOut,
    StateAutoInit
  }

  public static m_intakeStates m_intakeState;

  double stateTimer = 50;

  private final WPI_SparkMax m_spark = new WPI_SparkMax(kIntakeSparkPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_spark.getEncoder();
  private final SR_SimpleMotorFeedforward m_feedforward = new SR_SimpleMotorFeedforward(kSIntakeVolts,
      kVIntakeVoltSecondsPerMeter, kAIntakeVoltSecondsSquaredPerMeter);

  @Log(name = "Velocity Setpoint")
  private double m_velocitySetpointRotationsPerSecond;

  private double m_lastVelocitySetpoint;

  @Log(name = "Feedforward output")
  private double m_feedforwardOutput;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  @Log(name = "output voltage")
  private double m_lastOutputVoltage = 0;

  public Intake() {
    // m_spark.kCompensatedNominalVoltage
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setIdleMode(IdleMode.kBrake);
    m_spark.setInverted(true);
    // maybe add current limiting questionmark

    m_intakeState = m_intakeStates.StateWait;

    m_spark.setSmartCurrentLimit(kIntakeSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kIntakeImmediateCurrentLimitAmps);
    // some people were saying that this method was not properly implemented and
    // does nothing
    // it probably works now as those complaints were from 2020 but check this if
    // there are issues

    // m_feedforward.(kIntakeVelocityToleranceRotationsPerSecond);
  }

  @Log(name = "Current Velocity (RPS)")
  public double getVelocityRotationsPerSecond() {
    return m_encoder.getVelocity() / (kIntakeGearRatio * Constants.kNeoSensorUnitsPerRotation);
  }

  @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_voltageOverride = false;

    m_velocitySetpointRotationsPerSecond = rotationsPerSecond;
  }

  @Config.NumberSlider(name = "setVoltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public void runControlLoop() {
    double outputVoltage = 0;

    if (m_voltageOverride) {
      outputVoltage = m_voltageSetpoint;
    } else {
      double feedForwardOutput = m_feedforward.calculate(m_lastVelocitySetpoint, m_velocitySetpointRotationsPerSecond,
          kIntakeLoopTimeSeconds);

      outputVoltage = feedForwardOutput;
      m_feedforwardOutput = feedForwardOutput;
    }

    if (outputVoltage != m_lastOutputVoltage) {
      m_spark.setVoltage(outputVoltage);
    }

    m_lastOutputVoltage = outputVoltage;
  }

  private m_intakeStates m_requestedState;

  public void requestState(m_intakeStates state) {
    m_requestedState = state;
  }

  @Override
  public void periodic() {

    switch (m_intakeState) {
      case StateFault:
        break;
      case StateWait:
        if (RobotState.isEnabled()) {
          m_intakeState = m_intakeStates.StateInit;
        }
        break;
      case StateInit:
        m_intakeState = m_intakeStates.StateOff;
        break;
      case StateOff:
        this.runControlLoop();
        this.setVelocityRotationsPerSecond(0);
        if (m_requestedState != null)
          m_intakeState = m_requestedState;
        m_requestedState = null;
        break;
      case StateOn:
        this.runControlLoop();
        this.setVelocityRotationsPerSecond(kIntakeDefaultSpeedRotationsPerSecond);
        if (m_requestedState != null)
          m_intakeState = m_requestedState;
        m_requestedState = null;
        break;
      case StateOut:
        this.runControlLoop();
        this.setVelocityRotationsPerSecond(-kIntakeDefaultSpeedRotationsPerSecond);
        if (m_requestedState != null)
          m_intakeState = m_requestedState;
        m_requestedState = null;
        break;
      case StateAutoInit:
        this.runControlLoop();
        this.setVelocityRotationsPerSecond(-kIntakeDefaultSpeedRotationsPerSecond);
        if(stateTimer==0){
          m_intakeState=m_intakeStates.StateOff;
        }else{
          stateTimer--;
        }
        if (m_requestedState != null)
          m_intakeState = m_requestedState;
        m_requestedState = null;
        break;
    }
  }
}
