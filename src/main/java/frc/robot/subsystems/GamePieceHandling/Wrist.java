package frc.robot.subsystems.GamePieceHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.WristConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.WarlordsLib.sendableRichness.SR_ArmFeedforward;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

public class Wrist extends SubsystemBase implements Loggable {
  private WPI_TalonFX m_talon = new WPI_TalonFX(kWristTalonPort);
  // private final WL_SparkMax m_spark = new WL_SparkMax(kWristSparkPort);

  private final SR_ProfiledPIDController m_controller = new SR_ProfiledPIDController(kPWrist, kIWrist, kDWrist,
      kWristMotionProfileConstraints);

  @Log(name = "Wrist Feedforward")
  private final SR_ArmFeedforward m_feedforward = new SR_ArmFeedforward(
      kSWristVolts, kGWristVolts, kVWristVoltSecondsPerRadian, kAWristVoltSecondsSquaredPerRadian);

  @Log(name = "angle setpoint radians")
  // sets starting angle as midpoint of wrist range
  private double m_angleSetpointRadiansCurrent = 0;

  private double m_angleSetpointRadiansFinal = m_angleSetpointRadiansCurrent;

  private double m_previousVelocitySetpoint = 0;

  private boolean m_isZeroed = false;

  public boolean m_voltageOverride = false;

  public boolean firstTime = true;

  public double filteredAngle;

  public double m_voltageSetpoint = 0;
  
  public boolean firstTime2 = true;



  @Log(name="timer")
  public double stateTimer = 0;

  public static enum m_wristStates {
    StateFault,
    StateWait,
    StateInit,
    StateZero,
    StateDown,
    StateBottom,
    StateMiddle,
    StateTop,
    StateHigh,
    StateIdle
  }

  public static m_wristStates m_wristState;
  public static m_wristStates m_requestedState;

  public Wrist() {
    m_wristState = m_wristStates.StateIdle;

    TalonFXConfiguration wristTalonConfig = new TalonFXConfiguration();
    wristTalonConfig.voltageCompSaturation = kNominalVoltage;
    wristTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    wristTalonConfig.velocityMeasurementWindow = 1;

    wristTalonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        true,
        kIndexerSupplyCurrentLimitAmps,
        kIndexerSupplyCurrentThresholdAmps,
        kIndexerSupplyCurrentThresholdTimeSecs);

    wristTalonConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        true,
        kIndexerStatorCurrentLimitAmps,
        kIndexerStatorCurrentThresholdAmps,
        kIndexerStatorCurrentThresholdTimeSecs);

    m_talon.configAllSettings(wristTalonConfig);
    m_talon.setNeutralMode(NeutralMode.Brake);
    m_talon.setInverted(false);
    m_talon.enableVoltageCompensation(true);
    m_talon.configNeutralDeadband(0.001);

    m_controller.setTolerance(0.0001);

    this.resetAngleRadians(0);

    Shuffleboard.getTab("Wrist").add("Wrist controller", m_controller);
    Shuffleboard.getTab("Wrist").add("Current State", m_wristState.name());
  }

  public void requestState(m_wristStates state) {
    m_requestedState = state;
  }

  @Log(name = "enabled")
  public boolean isEnabled(){
    return RobotState.isEnabled();
  }

  @Log(name = "error")
  public double getError() {
    return Math.abs(m_angleSetpointRadiansCurrent - this.getAngleRadians());
  }

  public boolean atSetpoint(){
    return this.getError() < kWristTolerance;
  }

  public double getSetpoint(){
    return m_angleSetpointRadiansCurrent;
  }

  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_talon.getSelectedSensorPosition() * kWristRadiansPerPulse;
  }

  public void setAngleRadians(double angle) {
    m_voltageOverride = false;
    m_angleSetpointRadiansCurrent = MathUtil.clamp(angle, kWristBottomPositionRadians, kWristTopPositionRadians);
  }

  @Log(name = "velocity")
  public double getVelocityRadiansPerSecond(){
    return m_talon.getSelectedSensorVelocity()*kWristRadiansPerPulse;
  }

  public void resetAngleRadians(double angle) {
    m_talon.setSelectedSensorPosition(angle / kWristRadiansPerPulse);
  }

  public void setVoltage(double voltage){
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public void runControlLoop(){
    if(m_voltageOverride){
      m_talon.set(ControlMode.PercentOutput, m_voltageSetpoint/kNominalVoltage);
    }else{

    if(firstTime){
      filteredAngle = this.getAngleRadians();
      firstTime = false;
    }else{
      filteredAngle += ((this.getAngleRadians()-filteredAngle) * 0.5);
    }
    double controllerVoltage = m_controller.calculate(filteredAngle, m_angleSetpointRadiansCurrent);

    double feedforwardVoltage = m_feedforward.calculate(
        m_angleSetpointRadiansCurrent,
        m_previousVelocitySetpoint,
        m_controller.getSetpoint().velocity,
        kTimestepSeconds);

    m_previousVelocitySetpoint = m_controller.getSetpoint().velocity;

    m_talon.set(ControlMode.PercentOutput, (controllerVoltage + feedforwardVoltage) / kNominalVoltage);
  }
  }

  @Override
  public void periodic() {
    switch (m_wristState) {
      case StateFault:
        break;
      case StateWait:
        if(RobotState.isEnabled()){
          m_wristState = m_wristStates.StateInit;
        }
        break;
      case StateInit:
        stateTimer = 50;
        m_wristState = m_wristStates.StateZero;
        break;
      case StateZero:
        m_talon.setVoltage(-0.5);
        if(stateTimer==0){
          if (Math.abs(this.getVelocityRadiansPerSecond()) < 0.01) {
            this.resetAngleRadians(0);
            this.setAngleRadians(1.8);
            m_talon.setVoltage(0);
            m_wristState = m_wristStates.StateIdle;
          }
        }else{
          stateTimer--;
        }
        break;
      case StateDown:
        this.setAngleRadians(0);
        m_wristState = m_wristStates.StateIdle;
      case StateBottom:
        this.setAngleRadians(1.35);
        m_wristState = m_wristStates.StateIdle;
        break;
      case StateMiddle:
        this.setAngleRadians(1.8);
        m_wristState = m_wristStates.StateIdle;
        break;
      case StateTop:
        this.setAngleRadians(2.2);
        m_wristState = m_wristStates.StateIdle;
        break;
      case StateHigh:
        this.setAngleRadians(2.65);
        m_wristState = m_wristStates.StateIdle;
        break;
      case StateIdle:
        this.runControlLoop();
        if (m_requestedState != null) m_wristState = m_requestedState;
        m_requestedState = null;

        if(firstTime2){
          if(Telescope.m_telescopeState==Telescope.m_telescopeStates.StateIdle){
            m_wristState = m_wristStates.StateInit;
            firstTime2 = false;
          }
        }
        break; 
    }   
}
}