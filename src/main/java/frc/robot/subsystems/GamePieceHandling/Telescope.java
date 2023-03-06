package frc.robot.subsystems.GamePieceHandling;

import static frc.robot.Constants.TelescopeConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.motorcontrol.base.WPI_SparkMax;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import frc.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.subsystems.GamePieceHandling.Elevator.m_elevatorStates;
import frc.robot.subsystems.GamePieceHandling.Gripper.m_gripperStates;
import frc.robot.subsystems.GamePieceHandling.Wrist.m_wristStates;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Telescope extends SubsystemBase implements Loggable {
  // private WPI_TalonFX m_spark = new WL_TalonFX(kTelescopePort);
  private WPI_SparkMax m_spark = new WPI_SparkMax(kTelescopePort, MotorType.kBrushless);

  private static final double kTelescopeStartPosition = 0;

  public double currentPosition = 0;
  public double lastPosition = 0;

  private double m_feedForwardVoltage, m_voltageSetpoint, m_lastVelocitySetpoint;

  @Log(name = "setpoint")
  private double m_positionSetpointMeters = 0;
  private double m_lastPositionMeters = 0;
  private double m_outputPercentage, m_feedbackOutput, m_feedforwardOutput;
  private boolean m_isEnabled, m_voltageOverride;

  @Log(name = "state timer")
  public int stateTimer = 0;

  @Log(name = "Current timer")
  public int currentTimer = 0;

  @Log(name = "position timer")
  public int positionTimer = 3;

  public boolean firstTime = true;

  private final SR_SimpleMotorFeedforward m_feedforward = new SR_SimpleMotorFeedforward(
      kSTelescopeVolts, kVTelescopeVoltSecondsPerMeter, kATelescopeVoltSecondsSquaredPerMeter);

  SR_ProfiledPIDController telescopeController = new SR_ProfiledPIDController(
      kPTelescope,
      kITelescope,
      kDTelescope,
      kMotionProfileConstraints);

  public static enum m_telescopeStates {
    StateFault,
    StateWait,
    StateInit,
    StateZero,
    StatePickup,
    StateIn,
    StateMiddleCube,
    StateMiddleCone,
    StateOutCube,
    StateOutCone,
    StateIdle,
    StateAutoWait,
    StateAutoInit,
    StateAutoIn
  }

  public static m_telescopeStates m_telescopeState;
  public static m_telescopeStates m_requestedState = null;

  public Telescope() {

    if (RobotState.isAutonomous()) {
      m_telescopeState = m_telescopeStates.StateAutoWait;
    } else {
      m_telescopeState = m_telescopeStates.StateWait;
    }
    // m_telescopeState = m_telescopeStates.StateFault;

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
    m_spark.setSecondaryCurrentLimit(kTelescopeImmediateCurrentLimitAmps);
    m_spark.setInverted(false);
    m_spark.setIdleMode(IdleMode.kBrake);

    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    this.resetPositionMeters(0.05);
    // m_spark.getEncoder().setMeasurementPeriod(64);

  }

  public static void requestState(m_telescopeStates state) {
    m_telescopeState = state;
  }

  public void setPositionSetpointMeters(double position) {
    m_voltageOverride = false;
    m_positionSetpointMeters = MathUtil.clamp(position, kTelescopeStartPosition, kTelescopeMaxPosition);
  }

  @Log(name = "error")
  public double getError() {
    return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
  }

  public boolean atSetpoint() {
    return this.getError() < kTelescopeTolerance;
  }

  @Log(name = "current")
  public double getCurrent() {
    return m_spark.getOutputCurrent();
  }

  @Log(name = "position")
  public double getPositionMeters() {
    return m_spark.getEncoder().getPosition() * kDistancePerMotorRev;
  }

  public void resetPositionMeters(double position) {
    m_spark.getEncoder().setPosition(position);
  }

  @Log(name = "velocity")
  public double getVelocityMetersPerSecond() {
    return m_spark.getEncoder().getVelocity() * kDistancePerMotorRev;
  }

  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;

  }

  public void runControlLoop() {
    if (m_voltageOverride) {
      m_spark.set(m_voltageSetpoint / Constants.kNominalVoltage);
    } else {
      double feedbackOutputVoltage = 0;

      currentPosition = this.getPositionMeters();
      if (currentPosition < 0.01) {
        currentPosition = lastPosition;
      }

      feedbackOutputVoltage = telescopeController.calculate(currentPosition, m_positionSetpointMeters);

      double feedForwardOutputVoltage = 0;

      feedForwardOutputVoltage = m_feedforward.calculate(m_lastVelocitySetpoint,
          telescopeController.getSetpoint().velocity, kTelescopeControlLoopTimeSeconds);

      // if(this.getPositionMeters()!=0){
      m_outputPercentage = (feedbackOutputVoltage + feedForwardOutputVoltage) / Constants.kNominalVoltage; // same deal
                                                                                                           // as above
      // }

      m_feedbackOutput = feedbackOutputVoltage;
      m_feedforwardOutput = feedForwardOutputVoltage;

      m_spark.set(m_outputPercentage);

      if (this.getPositionMeters() > 0.01) {
        lastPosition = currentPosition;
      }
      m_lastVelocitySetpoint = telescopeController.getSetpoint().velocity;
    }
  }

  @Override
  public void periodic() {

    switch (m_telescopeState) {
      case StateFault:
        break;
      case StateWait:
        if (RobotState.isEnabled()) {
          m_telescopeState = m_telescopeStates.StateInit;
        }
        break;
      case StateInit:
        stateTimer = 20;
        currentTimer = 5;
        m_telescopeState = m_telescopeStates.StateZero;
        break;
      case StateZero:
        m_spark.setVoltage(-1.5);
        if (this.getCurrent() > 40) {
          currentTimer--;
          if (currentTimer <= 0) {
            // calling the zero 0.05 to avoid running into the random position drops
            this.resetPositionMeters(0.05);
            this.setPositionSetpointMeters(0.15);
            m_spark.setVoltage(0);
            m_telescopeState = m_telescopeStates.StateIdle;
          }
        } else {
          currentTimer = 5;
        }

        break;
      case StatePickup:
        this.setPositionSetpointMeters(0.03);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateIn:
        this.setPositionSetpointMeters(0.15);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateMiddleCube:
        this.setPositionSetpointMeters(0.55);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateOutCube:
        this.setPositionSetpointMeters(0.87);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateMiddleCone:
        this.setPositionSetpointMeters(0.55);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateOutCone:
        this.setPositionSetpointMeters(0.98);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateIdle:
        this.runControlLoop();
        if (m_requestedState != null)
          m_telescopeState = m_requestedState;
        m_requestedState = null;
        break;

      case StateAutoWait:
        if (m_requestedState != null)
          m_telescopeState = m_requestedState;
        m_requestedState = null;
        break;
      case StateAutoInit:
        this.setPositionSetpointMeters(0.87);
        if (RobotState.isEnabled()) {
          this.runControlLoop();
        }
        if (this.atSetpoint()) {
          if (firstTime) {
            stateTimer = 25;
            firstTime = false;
          }
          Gripper.requestState(m_gripperStates.StateInit);
          if (stateTimer == 0) {
            m_telescopeState = m_telescopeStates.StateAutoIn;
            firstTime = true;
          } else {
            stateTimer--;
          }
        }
        if (m_requestedState != null)
          m_telescopeState = m_requestedState;
        m_requestedState = null;
        break;
      case StateAutoIn:
        if (firstTime) {
          stateTimer = 50;
          firstTime = false;
        }
        this.setPositionSetpointMeters(0);
        if (stateTimer == 0) {
          Elevator.requestState(m_elevatorStates.StateInit);
          Wrist.requestState(m_wristStates.StateAutoBottom);
          stateTimer--;
        } else {
          stateTimer--;
        }
        this.runControlLoop();
        if (m_requestedState != null)
          m_telescopeState = m_requestedState;
        m_requestedState = null;
        break;
    }
  }
}
