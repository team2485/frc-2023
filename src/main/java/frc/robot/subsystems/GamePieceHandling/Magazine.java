package frc.robot.subsystems.GamePieceHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MagazineConstants.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Magazine extends SubsystemBase implements Loggable {

  private WPI_TalonFX m_talon = new WPI_TalonFX(kMagazineTalonPort);

  private final SR_SimpleMotorFeedforward m_feedforward = new SR_SimpleMotorFeedforward(
      kSMagazineVolts, kVMagazineVoltSecondsPerMeter, kAMagazineVoltSecondsSquaredPerMeter);

  @Log(name = "Velocity Setpoint")
  private double m_velocitySetpointRotationsPerSecond;

  private double m_lastVelocitySetpoint = 0;

  private double m_lastVelocity;

  // @Log(name = "Feedforward output")
  private double m_feedforwardOutput;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  private double m_lastOutputVoltage = 0;

  @Log(name = "output voltage")
  private double m_outputVoltage = 0;

  public enum m_magazineStates {
    StateFault,
    StateWait,
    StateInit,
    StateOn,
    StateOff,
    StateOut
  }

  private m_magazineStates m_magazineState;

  private m_magazineStates m_requestedState;

  public void requestState(m_magazineStates state) {
    m_requestedState = state;
  }

  public Magazine() {
    m_magazineState = m_magazineStates.StateFault;

    TalonFXConfiguration magazineTalonConfig = new TalonFXConfiguration();
    magazineTalonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    magazineTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    magazineTalonConfig.velocityMeasurementWindow = 1;

    magazineTalonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        true,
        kMagazineSupplyCurrentLimitAmps,
        kMagazineSupplyCurrentThresholdAmps,
        kMagazineSupplyCurrentThresholdTimeSecs);

    magazineTalonConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        true,
        kMagazineStatorCurrentLimitAmps,
        kMagazineStatorCurrentThresholdAmps,
        kMagazineStatorCurrentThresholdTimeSecs);

    m_talon.configAllSettings(magazineTalonConfig);
    m_talon.setNeutralMode(NeutralMode.Brake);
    m_talon.setInverted(false);
    m_talon.enableVoltageCompensation(true);
  }

  /** @return the current velocity in rotations per second. */
  // @Log(name = "Current velocity (RPS)")
  @Log(name = "current rotations/sec")
  public double getVelocityRotationsPerSecond() {
    return m_talon.getSelectedSensorVelocity()
        / (kMagazineGearRatio * kFalconSensorUnitsPerRotation);
  }

  /**
   * Sets the velocity setpoint for the feeeder.
   *
   * @param rotationsPerSecond velocity setpoint
   */
  // @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_voltageOverride = false;
    m_velocitySetpointRotationsPerSecond = rotationsPerSecond;
  }

  /**
   * Applys the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
  // @Config.NumberSlider(name = "Set voltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  // @Log(name = "Stator current")
  // public double getStatorCurrent() {
  // return m_talon.getOutputCurrent();
  // }

  // @Log(name = "At setpoint")
  public boolean atSetpoint() {
    return Math.abs(getVelocityRotationsPerSecond()
        - m_velocitySetpointRotationsPerSecond) < kMagazineVelocityToleranceRotationsPerSecond;
  }

  public void runControlLoop() {
    // Calculates voltage to apply.
    m_outputVoltage = 0;

    if (m_voltageOverride) {
      m_outputVoltage = m_voltageSetpoint;
    } else {
      double feedforwardOutput = m_feedforward.calculate(
          m_lastVelocitySetpoint,
          m_velocitySetpointRotationsPerSecond,
          kMagazineLoopTimeSeconds);

      m_outputVoltage = feedforwardOutput;

      m_feedforwardOutput = feedforwardOutput;
    }

    if (m_outputVoltage != m_lastOutputVoltage) {
      m_talon.setVoltage(m_outputVoltage);
    }

    m_lastOutputVoltage = m_outputVoltage;
    m_lastVelocity = this.getVelocityRotationsPerSecond();
  }

  public void periodic() {
    switch (m_magazineState) {
      case StateFault:
        break;
      case StateWait:
        if (RobotState.isEnabled()) {
          m_magazineState = m_magazineStates.StateInit;
        }
        break;
      case StateInit:
        m_magazineState = m_magazineStates.StateOff;
        break;
      case StateOff:
        // this.setVoltage(m_lastOutputVoltage);
        this.runControlLoop();
        this.setVelocityRotationsPerSecond(0);
        if (m_requestedState != null)
          m_magazineState = m_requestedState;
        m_requestedState = null;
        break;
      case StateOn:
        this.runControlLoop();
        this.setVelocityRotationsPerSecond(kMagazineDefaultSpeedRotationsPerSecond);
        if (m_requestedState != null)
          m_magazineState = m_requestedState;
        m_requestedState = null;
        break;
      case StateOut:
        this.runControlLoop();
        this.setVelocityRotationsPerSecond(-6);
        if (m_requestedState != null)
          m_magazineState = m_requestedState;
        m_requestedState = null;
        break;
    }
  }

}
