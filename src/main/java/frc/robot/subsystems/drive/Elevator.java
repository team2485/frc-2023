package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants;

public class Elevator {
    private double feedForwardVoltage = 0;
    private double m_positionSetpointMeters = 0;
    private double m_voltageSetpoint = 0;
    private boolean m_enables = false;

    private final WPI_TalonFX m_talonTop = new WPI_TalonFX(ElevatorConstants.kElevatorPortTop);
    private final WPI_TalonFX m_talonBottom = new WPI_TalonFX(ElevatorConstants.kElevatorPortBottom);

    private final ElevatorFeedforward m_feedforwardUnloaded = new ElevatorFeedforward(ElevatorConstants.kSElevatorUnloadedVolts, ElevatorConstants.kGElevatorUnloadedVolts, ElevatorConstants.kVElevatorVoltsSecondsPerMeter, ElevatorConstants.kAElevatorVoltsSecondsSquaredPerMeter);
    private final ElevatorFeedforward m_feedforwardLoaded = new ElevatorFeedforward(ElevatorConstants.kSElevatorLoadedVolts, ElevatorConstants.kGElevatorLoadedVolts, ElevatorConstants.kVElevatorVoltsSecondsPerMeter, ElevatorConstants.kAElevatorVoltsSecondsSquaredPerMeter);
    
    public void setPositionMeters(double position){
        m_positionSetpointMeters = MathUtil.clamp(position, ElevatorConstants.kElevatorBottomStop, ElevatorConstants.kElevatorTopStop);
    }
    
    public double getError() {
        return Math.abs(m_positionSetpointMeters - this.getPositionMeters)
    }
    
    public double getPositionMeters() {
        return m_sensor; //Once proper sensor is determined and created, replace this temperary code
    }
    
    //public void ControlLoop() {
    
    //}
}