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
import frc.WarlordsLib.sendableRichness.*;
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

  private Wrist m_wrist = new Wrist();

  private final WPI_TalonFX m_talonLeft = new WPI_TalonFX(kElevatorPortLeft);
  private final WPI_TalonFX m_talonRight = new WPI_TalonFX(kElevatorPortRight);

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
    StateWait,
    StateInit,
    StateZero,
    StateBottom,
    StateMiddle,
    StateTop,
    StateIdle
  }

  public static m_elevatorStates m_elevatorState;
  public static m_elevatorStates m_requestedState;

  private double stateTimer = 0;

  public Elevator() {
    m_elevatorState = m_elevatorStates.StateWait;

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

    m_pidController.setTolerance(0.05);

    this.resetPositionMeters(0);

  }

  public void setPositionMeters(double position) {
    m_voltageOverride = false;
    m_positionSetpointMeters = MathUtil.clamp(position, kElevatorBottomStop, kElevatorTopStop);
  }

  public double getError() {
    return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
  }

  @Log(name = "at setpoint")
  public boolean atSetpoint() {
    return m_pidController.atSetpoint();
  }

  @Log(name = "position")
  public double getPositionMeters() {
    return m_talonLeft.getSelectedSensorPosition() * kDistancePerPulse;
  }

  public void resetPositionMeters(double position) {
    m_talonLeft.setSelectedSensorPosition(position / kDistancePerPulse);
    m_talonRight.setSelectedSensorPosition(position / kDistancePerPulse);
  }

  public double getVelocityMetersPerSecond() {
    return m_talonLeft.getSelectedSensorVelocity() * kDistancePerPulse;
  }

  @Log(name = "output voltage")
  public double getVoltage() {
    return m_talonLeft.getMotorOutputVoltage();
  }

  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public void runControlLoop() {
    if (m_voltageOverride) {
      m_talonLeft.set(ControlMode.PercentOutput, m_voltageSetpoint / kNominalVoltage);
      m_talonRight.set(ControlMode.PercentOutput, m_voltageSetpoint / kNominalVoltage);
    } else {

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
  }

  public void requestState(m_elevatorStates state) {
    m_requestedState = state;
  }

  @Override
  public void periodic() {
    switch (m_elevatorState) {
      case StateFault:
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
        m_talonLeft.setVoltage(-0.75);
        m_talonRight.setVoltage(-0.75);
        if (stateTimer == 0) {
          if (Math.abs(this.getVelocityMetersPerSecond()) < 0.01) {
            this.resetPositionMeters(0);
            this.setPositionMeters(0);
            m_talonLeft.setVoltage(0);
            m_talonRight.setVoltage(0);
            m_elevatorState = m_elevatorStates.StateIdle;
          }
        } else {
          stateTimer--;
        }
        break;
      case StateBottom:
        m_wrist.requestState(m_wristStates.StateMiddle);
        this.setPositionMeters(0);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateMiddle:
        // TODO: update setpoint value
        this.setPositionMeters(0.5);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateTop:
        // TODO: update setpoint value
        this.setPositionMeters(1);
        m_elevatorState = m_elevatorStates.StateIdle;
        break;
      case StateIdle:
        if (m_voltageOverride) {
          m_talonLeft.set(m_voltageSetpoint / kNominalVoltage);
        } else {
          this.runControlLoop();
        }

        if (m_requestedState != null)
          m_elevatorState = m_requestedState;
        m_requestedState = null;
        break;
    }
  }

}
