package frc.robot.subsystems.drive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import static frc.robot.Constants.ElevatorConstants.*;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Elevator extends SubsystemBase implements Loggable{
    private double feedForwardVoltage = 0;
    private double m_positionSetpointMeters = 0;
    private double m_voltageSetpoint = 0;
    private boolean m_enables = false;
    private boolean m_voltageOverride = false;

    private double m_lastVelocitySetpoint = 0;
    private double talonOutputPercentage; //reconsider naming, vaguely copied over

    private double m_feedbackOutput; //reconsider naming, vaguely copied over
    private double m_feedforwardOutput; //reconsider naming, vaguely copied over

    private final WPI_TalonFX m_talonLeft = new WPI_TalonFX(kElevatorPortLeft);
    private final WPI_TalonFX m_talonRight = new WPI_TalonFX(kElevatorPortRight);

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward( kSElevatorVolts,  kGElevatorVolts,  kVElevatorVoltsSecondsPerMeter,  kAElevatorVoltsSecondsSquaredPerMeter);
    
    private final ProfiledPIDController m_pidController = 
        new ProfiledPIDController(
            kPElevatorVoltsPerMeter, kIElevatorVoltsPerMeter, 
            kDElevatorVoltSecondsPerMeter, 
            kElevatorControllerConstraints, 
            kElevatorControlLoopTimeSeconds);

    public Elevator(){
        TalonFXConfiguration talonConfiguration = new TalonFXConfiguration();
        talonConfiguration.voltageCompSaturation = Constants.kNominalVoltage;
        talonConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, kElevatorSupplyCurrentLimit, kElevatorSupplyThresholdCurrent, kElevatorSupplyThresholdTime);
        talonConfiguration.statorCurrLimit = new StatorCurrentLimitConfiguration(true, kElevatorStatorCurrentLimit, kElevatorStatorThresholdCurrent, kElevatorStatorThresholdTime);

        m_talonLeft.configAllSettings(talonConfiguration);
        m_talonLeft.setNeutralMode(NeutralMode.Brake);
        m_talonRight.configAllSettings(talonConfiguration);
        m_talonRight.setNeutralMode(NeutralMode.Brake);
        //set inverted if inverted
        m_pidController.setTolerance(kElevatorPositionTolerance, kElevatorVelocityTolerance);
    }

    public void setPositionMeters(double position){
        m_voltageOverride = false;
        m_positionSetpointMeters = MathUtil.clamp(position,  kElevatorBottomStop,  kElevatorTopStop);
    }
    
    public double getPositionMeters() {
        return m_talonLeft.getSelectedSensorPosition() * kDistancePerPulse;
    }

    public double getError() {
        return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
    }

    public double getVelocityMetersPerSecond() {
        return m_talonLeft.getSelectedSensorVelocity() * kDistancePerPulse;
    }

    public void setVoltage(double voltage) {
        m_voltageOverride = true;
        m_voltageSetpoint = voltage;
    }

    public double getVoltage(){
        return m_talonLeft.getMotorOutputVoltage();
    }

    public void runControlLoop() {
        if (m_voltageOverride) {
            m_talonLeft.set(ControlMode.PercentOutput, m_voltageSetpoint / kNominalVoltage);
            m_talonRight.set(ControlMode.PercentOutput, m_voltageSetpoint / kNominalVoltage);
        }
        else { 

        double feedbackOutputVoltage = 0;

            feedbackOutputVoltage =
                m_pidController.calculate(this.getPositionMeters(), m_positionSetpointMeters);

        double feedforwardOutputVoltage = 0;

            feedforwardOutputVoltage =
                m_feedforward.calculate(
                    m_lastVelocitySetpoint,
                    m_pidController.getSetpoint().velocity); //time factor?
                
        talonOutputPercentage =
            (feedbackOutputVoltage + feedforwardOutputVoltage) / kNominalVoltage;

        m_feedbackOutput = feedbackOutputVoltage;
        m_feedforwardOutput = feedforwardOutputVoltage;
    
        m_talonLeft.set(ControlMode.PercentOutput, talonOutputPercentage);
        m_talonRight.set(ControlMode.PercentOutput, talonOutputPercentage);

          m_lastVelocitySetpoint = m_pidController.getSetpoint().velocity;
        }
    }
    
    public void periodic(){
        this.runControlLoop();
    }
}