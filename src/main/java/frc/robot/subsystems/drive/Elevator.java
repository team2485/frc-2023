package frc.robot.subsystems.drive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.ElevatorConstants.*;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.WarlordsLib.sendableRichness.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements Loggable{
    private double feedForwardVoltage = 0;
    private double m_positionSetpointMeters = 0;
    private double m_voltageSetpoint = 0;
    private boolean m_enables = false;
    private boolean m_voltageOverride = false;

    private final WPI_TalonFX m_talonTop = new WPI_TalonFX(kElevatorPortTop);
    private final WPI_TalonFX m_talonBottom = new WPI_TalonFX(kElevatorPortBottom);

    private final SR_ElevatorFeedforward m_feedforward = new SR_ElevatorFeedforward( kSElevatorVolts,  kGElevatorVolts,  kVElevatorVoltsSecondsPerMeter,  kAElevatorVoltsSecondsSquaredPerMeter);
    
    private final SR_ProfiledPIDController m_pidController = 
        new SR_ProfiledPIDController(
            kPElevatorVoltsPerMeter, 
            0, 
            kDElevatorVoltSecondsPerMeter, 
            kElevatorControllerConstraints, 
            kElevatorControlLoopTimeSeconds);

    public boolean isInverted() {
        return m_talonTop.getInverted();
    }
    
    public void invertTalon() {
        if (m_talonTop.getInverted()) {
            m_talonTop.setInverted(false);
            m_talonBottom.setInverted(false);
        } else {
            m_talonTop.setInverted(true);
            m_talonBottom.setInverted(true);
        }
    }

    public void setPositionMeters(double position){
        m_voltageOverride = false;
        m_positionSetpointMeters = MathUtil.clamp(position,  kElevatorBottomStop,  kElevatorTopStop);
    }
    
    public double getError() {
        return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
    }
    
    public double getPositionMeters() {
        return m_talonTop.getSelectedSensorPosition();
    }

    public void resetPositionMeters(double position) {
        m_talonTop.setSelectedSensorPosition(position);
        m_talonBottom.setSelectedSensorPosition(position);
    }

    public double getVelocityMetersPerSecond() {
        return m_talonTop.getSelectedSensorVelocity();
    }

    
    public void setVoltage(double voltage) {
        m_voltageOverride = true;
        m_voltageSetpoint = voltage;
    }

    public void runControlLoop() {
        /*if (m_enabled) {
            if (m_voltageOverride) {
                m_talon.set(ControlMode.PercentOutput, m_voltageSetpoint / Constants.kNominalVoltage);
            } else {
            double feedbackOutputVoltage = 0;
    
            if (m_loaded) {
              feedbackOutputVoltage =
                  m_pidControllerLoaded.calculate(this.getPositionMeters(), m_positionSetpointMeters);
            } else {
              feedbackOutputVoltage =
                  m_pidControllerUnloaded.calculate(this.getPositionMeters(), m_positionSetpointMeters);
            }
    
            double feedforwardOutputVoltage = 0;
            if (m_loaded) {
              feedforwardOutputVoltage =
                  m_feedforwardLoaded.calculate(
                      m_lastVelocitySetpoint,
                      m_pidControllerLoaded.getSetpoint().velocity,
                      kElevatorControlLoopTimeSeconds);
            } else {
              feedforwardOutputVoltage =
                  m_feedforwardUnloaded.calculate(
                      m_lastVelocitySetpoint,
                      m_pidControllerUnloaded.getSetpoint().velocity,
                      kElevatorControlLoopTimeSeconds);
            }
    
            outputPercentage =
                (feedbackOutputVoltage + feedforwardOutputVoltage) / Constants.kNominalVoltage;
    
            if (!m_limitOverride) {
              // outputPercentage = this.limitOnSlotSensors(outputPercentage);
            }
    
            m_feedbackOutput = feedbackOutputVoltage;
            m_feedforwardOutput = feedforwardOutputVoltage;
            // m_talon.set(ControlMode.PercentOutput, limitOnSlotSensors(outputPercentage));
    
            m_talon.set(ControlMode.PercentOutput, outputPercentage);
    
            if (m_loaded) {
              m_lastVelocitySetpoint = m_pidControllerLoaded.getSetpoint().velocity;
            } else {
              m_lastVelocitySetpoint = m_pidControllerUnloaded.getSetpoint().velocity;
            }
          }
        } else {
          m_talon.set(ControlMode.PercentOutput, 0);
        }*/
    }
}