package frc.robot.subsystems.GamePieceHandling;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import static frc.robot.Constants.ElevatorConstants.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
//import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.WarlordsLib.sendableRichness.*;
import frc.robot.subsystems.GamePieceHandling.Telescope.m_telescopeStates;
import frc.robot.subsystems.GamePieceHandling.Wrist.m_wristStates;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Elevator extends SubsystemBase implements Loggable {
  private double feedForwardVoltage = 0;

  @Log(name = "setpoint")
  private double m_positionSetpointMeters = 0;

  private double m_voltageSetpoint = 0;
  private boolean m_enables = false;
  private boolean m_voltageOverride = false;

  private double m_lastVelocitySetpoint = 0;
  private double outputPercentage; // reconsider naming, vaguely copied over

  private double m_feedbackOutput = 0; // reconsider naming, vaguely copied over
  private double m_feedforwardOutput = 0; // reconsider naming, vaguely copied over

  private final WPI_TalonFX m_talonLeft = new WPI_TalonFX(kElevatorPortLeft);
  private final WPI_TalonFX m_talonRight = new WPI_TalonFX(kElevatorPortRight);

  public boolean firstTime = true;
  private boolean setpointOvershot = false;

  private final SR_ElevatorFeedforward m_feedforward = new SR_ElevatorFeedforward(kSElevatorVolts, kGElevatorVolts,
      kVElevatorVoltsSecondsPerMeter, kAElevatorVoltsSecondsSquaredPerMeter);

  private final SR_ProfiledPIDController m_pidController = new SR_ProfiledPIDController(
      kPElevatorVoltsPerMeter,
      kIElevatorVoltsPerMeter,
      kDElevatorVoltSecondsPerMeter,
      kElevatorControllerConstraints,
      kElevatorControlLoopTimeSeconds);

  public enum m_elevatorStates {
    StateFault,
    StateStart,
    StateWait,
    StateInit,
    StateZero,
    StateTravel,
    StateBottom,
    StateLow,
    StateMiddleCube,
    StateTopCube,
    StateMiddleCone,
    StateTopCone,
    StateIdle,
    StateAutoWait,
    StateAutoInit
  }

  public static m_elevatorStates m_elevatorState;
  public static m_elevatorStates m_requestedState;

  private double stateTimer = 0;

  public Elevator() {

    m_pidController.setIntegratorRange(0, 12);

    if (RobotState.isAutonomous()) {
      m_elevatorState = m_elevatorStates.StateAutoWait;
    } else {
      m_elevatorState = m_elevatorStates.StateStart;
    }

    // m_elevatorState = m_elevatorStates.StateFault;

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.voltageCompSaturation = kNominalVoltage;
    talonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        true,
        kElevatorSupplyCurrentLimitAmps,
        kElevatorSupplyCurrentThresholdAmps,
        kElevatorSupplyCurrentThresholdTimeSecs);
    talonConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        true,
        kElevatorStatorCurrentLimitAmps,
        kElevatorStatorCurrentThresholdAmps,
        kElevatorStatorCurrentThresholdTimeSecs);

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

    m_pidController.setTolerance(0);

    this.resetPositionMeters(0);
  }

  public void setPositionMeters(double position) {
    m_voltageOverride = false;
    m_positionSetpointMeters = MathUtil.clamp(position, kElevatorBottomStop, kElevatorTopStop);
  }

  @Log(name = "error")
  public double getError() {
    return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
  }

  @Log(name = "at setpoint")
  public boolean atSetpoint() {
    return this.getError() < kElevatorTolerance;
  }

  @Log(name = "position")
  public double getPositionMeters() {
    return m_talonLeft.getSelectedSensorPosition() * kDistancePerPulse;
  }

  @Log(name = "position2")
  public double getPositionMetersTheOtherOne() {
    return m_talonRight.getSelectedSensorPosition() * kDistancePerPulse;
  }

  public void resetPositionMeters(double position) {
    m_talonLeft.setSelectedSensorPosition(position / kDistancePerPulse);
    m_talonRight.setSelectedSensorPosition(position / kDistancePerPulse);
  }

  @Log(name = "velocity")
  public double getVelocityMetersPerSecond() {
    return m_talonLeft.getSelectedSensorVelocity() * kDistancePerPulse;
  }

  @Log(name = "output current")
  public double getStatorCurrent() {
    return m_talonLeft.getStatorCurrent();
  }

  @Log(name = "output current 2")
  public double getStatorCurrent2() {
    return m_talonRight.getStatorCurrent();
  }

  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public void runControlLoop() {
    // if(firstTime){
    // if(RobotState.isEnabled()){
    // m_pidController.reset(0);
    // firstTime=false;
    // }
    // }

    if (setpointOvershot && this.atSetpoint()) {
      setpointOvershot = false;
      m_positionSetpointMeters -= kElevatorOvershootAmountMeters;
    }

    double feedbackOutputVoltage = 0;

    feedbackOutputVoltage = m_pidController.calculate(this.getPositionMeters(), m_positionSetpointMeters);

    double feedforwardOutputVoltage = 0;

    feedforwardOutputVoltage = m_feedforward.calculate(
        m_lastVelocitySetpoint,
        m_pidController.getSetpoint().velocity,
        kElevatorControlLoopTimeSeconds);

    outputPercentage = (feedbackOutputVoltage + feedforwardOutputVoltage) / kNominalVoltage;

    m_feedbackOutput = feedbackOutputVoltage;
    m_feedforwardOutput = feedforwardOutputVoltage;

    m_talonLeft.set(ControlMode.PercentOutput, outputPercentage);
    m_talonRight.set(ControlMode.PercentOutput, outputPercentage);

    m_lastVelocitySetpoint = m_pidController.getSetpoint().velocity;
  }

  public static void requestState(m_elevatorStates state) {
    m_requestedState = state;
  }

  @Override
  public void periodic() {
    switch (m_elevatorState) {
      case StateFault:
        break;
      case StateStart:
        this.setPositionMeters(0.25);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateWait:
        if (RobotState.isEnabled())
          m_elevatorState = m_elevatorStates.StateInit;
        break;
      case StateInit:
        stateTimer = 25;
        m_elevatorState = m_elevatorStates.StateZero;
        break;
      case StateZero:
        m_talonLeft.setVoltage(-1.5);
        m_talonRight.setVoltage(-1.5);
        if (stateTimer == 0) {
          if (Math.abs(this.getVelocityMetersPerSecond()) < 0.03) {
            this.setPositionMeters(0.01);
            this.resetPositionMeters(0);
            m_pidController.reset(0);
            m_talonLeft.setVoltage(0);
            m_talonRight.setVoltage(0);
            m_elevatorState = m_elevatorStates.StateIdle;
            stateTimer--;
          }
        } else {
          stateTimer--;
        }
        break;
      case StateTravel:
        setpointOvershot = true;
        // this.setPositionMeters(0.25+kElevatorOvershootAmountMeters);
        this.setPositionMeters(0.315 + kElevatorOvershootAmountMeters);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateBottom:
        // setpointOvershot = true;
        // this.setPositionMeters(0+kElevatorOvershootAmountMeters);
        this.setPositionMeters(0.01);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateLow:
        // setpointOvershot = true;
        // this.setPositionMeters(0.25+kElevatorOvershootAmountMeters);
        this.setPositionMeters(0.315);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateMiddleCube:
        setpointOvershot = true;
        // this.setPositionMeters(0.5+kElevatorOvershootAmountMeters);
        this.setPositionMeters(0.54 + kElevatorOvershootAmountMeters);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateTopCube:
        setpointOvershot = true;
        // this.setPositionMeters(0.8+kElevatorOvershootAmountMeters);
        this.setPositionMeters(0.86);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateMiddleCone:
        setpointOvershot = true;
        // this.setPositionMeters(0.70+kElevatorOvershootAmountMeters);
        this.setPositionMeters(0.7 + kElevatorOvershootAmountMeters);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateTopCone:
        // setpointOvershot = true;
        // this.setPositionMeters(0.87+kElevatorOvershootAmountMeters);
        this.setPositionMeters(0.86);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateIdle:
        if (RobotState.isEnabled()) {
          this.runControlLoop();
        }

        if (m_requestedState != null)
          m_elevatorState = m_requestedState;
        m_requestedState = null;
        break;
      case StateAutoWait:
        m_pidController.reset(0);
        if (m_requestedState != null) {
          m_elevatorState = m_requestedState;
          stateTimer = 50;
        }
        m_requestedState = null;
        break;

      case StateAutoInit:
        this.setPositionMeters(0.66);
        if (RobotState.isEnabled()) {
          this.runControlLoop();
        }
        if (stateTimer == 0) {
          Telescope.requestState(m_telescopeStates.StateAutoInit);
          Wrist.requestState(m_wristStates.StateAutoInit);
          stateTimer--;
        } else {
          stateTimer--;
        }
        if (m_requestedState != null)
          m_elevatorState = m_requestedState;
        m_requestedState = null;
        break;
    }
  }
}
