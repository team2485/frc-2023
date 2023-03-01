package frc.robot.subsystems.GamePieceHandling;

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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.WarlordsLib.motorcontrol.base.WPI_SparkMax;
import frc.WarlordsLib.sendableRichness.SR_ElevatorFeedforward;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import frc.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import frc.WarlordsLib.sendableRichness.SR_TrapezoidProfile;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Telescope extends SubsystemBase implements Loggable{
  // private WPI_TalonFX m_spark = new WL_TalonFX(kTelescopePort);
  private WPI_SparkMax m_spark = new WPI_SparkMax(kTelescopePort, MotorType.kBrushless);

  private static final double kTelescopeStartPosition = 0;

  private double m_feedForwardVoltage, m_voltageSetpoint, m_lastVelocitySetpoint;

  @Log(name = "setpoint")
  private double m_positionSetpointMeters;
  private double m_outputPercentage, m_feedbackOutput, m_feedforwardOutput;
  private boolean m_isEnabled, m_voltageOverride;

  public int stateTimer = 0;

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
        StateIn,
        StateMiddleCube,
        StateMiddleCone,
        StateOutCube,
        StateOutCone,
        StateIdle
      }
  
    public static m_telescopeStates m_telescopeState;
    public static m_telescopeStates m_requestedState = null;


  public Telescope() {
    m_telescopeState = m_telescopeStates.StateWait;

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

  }

  public void requestState(m_telescopeStates state){
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


  public boolean atSetpoint(){
    return this.getError() < kTelescopeTolerance;
  }

  @Log(name="current")
  public double getCurrent(){
    return m_spark.getOutputCurrent();
  }

  @Log(name="position")
  public double getPositionMeters() {
    return m_spark.getEncoder().getPosition() * kDistancePerMotorRev;
  }

  public void resetPositionMeters(double position) {
    m_spark.getEncoder().setPosition(position);
  }

  @Log(name="velocity")
  public double getVelocityMetersPerSecond() {
    return m_spark.getEncoder().getVelocity() * kDistancePerMotorRev;
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
  
    switch(m_telescopeState){
      case StateFault:
        break;
      case StateWait:
        if(RobotState.isEnabled()){
          m_telescopeState = m_telescopeStates.StateInit;
        }
        break;
      case StateInit:
        stateTimer = 20;
        m_telescopeState = m_telescopeStates.StateZero;
        break;
      case StateZero:
        m_spark.setVoltage(-1.25);
        if(stateTimer==0){
          if (Math.abs(this.getCurrent()) > 30) {
            this.resetPositionMeters(-0.05);
            this.setPositionSetpointMeters(0.15);
            m_spark.setVoltage(0);
            m_telescopeState = m_telescopeStates.StateIdle;
          }
        }else{
          stateTimer--;
        }
        break;
      case StateIn:
        this.setPositionSetpointMeters(0.05);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateMiddleCube: 
        this.setPositionSetpointMeters(0.5);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateOutCube:
        this.setPositionSetpointMeters(0.8);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateMiddleCone: 
        this.setPositionSetpointMeters(0.5);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateOutCone:
        this.setPositionSetpointMeters(0.80);
        m_telescopeState = m_telescopeStates.StateIdle;
        break;
      case StateIdle: 
        this.runControlLoop();
        if(m_requestedState!=null) m_telescopeState = m_requestedState;
        m_requestedState = null;
        break;
    }
    
  }
}