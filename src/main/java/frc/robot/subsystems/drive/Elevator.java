package frc.robot.subsystems.drive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import static frc.robot.Constants.ElevatorConstants.*;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.WarlordsLib.sendableRichness.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Elevator extends SubsystemBase implements Loggable{
    private double feedForwardVoltage = 0;
    private double m_positionSetpointMeters = 0;
    private double m_voltageSetpoint = 0;
    private boolean m_enables = false;
    private boolean m_voltageOverride = false;

    private double m_lastVelocitySetpoint = 0;
    private double outputPercentage; //reconsider naming, vaguely copied over

    private double m_feedbackOutput = 0; //reconsider naming, vaguely copied over
    private double m_feedforwardOutput = 0; //reconsider naming, vaguely copied over

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
        if (m_voltageOverride) {
            m_talonTop.set(ControlMode.PercentOutput, m_voltageSetpoint / kNominalVoltage);
            m_talonBottom.set(ControlMode.PercentOutput, m_voltageSetpoint / kNominalVoltage);
        } 
        else {

        double feedbackOutputVoltage = 0;

            feedbackOutputVoltage =
                m_pidController.calculate(this.getPositionMeters(), m_positionSetpointMeters);

        double feedforwardOutputVoltage = 0;

            feedforwardOutputVoltage =
                m_feedforward.calculate(
                    m_lastVelocitySetpoint,
                    m_pidController.getSetpoint().velocity,
                    kElevatorControlLoopTimeSeconds);

        outputPercentage =
            (feedbackOutputVoltage + feedforwardOutputVoltage) / kNominalVoltage;

        m_feedbackOutput = feedbackOutputVoltage;
        m_feedforwardOutput = feedforwardOutputVoltage;
    
        m_talonTop.set(ControlMode.PercentOutput, outputPercentage);
        m_talonBottom.set(ControlMode.PercentOutput, outputPercentage);

          m_lastVelocitySetpoint = m_pidController.getSetpoint().velocity;
        }
    } 
}