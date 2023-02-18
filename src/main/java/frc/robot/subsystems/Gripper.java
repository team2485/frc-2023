package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import frc.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.robot.Constants;
import frc.util.BufferZone;
/**
 * A gripper subsystem. Gripper is controlled by a NEO. 
 * Heavily derrived from ncsariowan and mark-rifkin's Hood subsystem for FRC-2019
 * 
 * 
 */
public class Gripper extends SubsystemBase implements Loggable {
    
    private CANSparkMax m_motor;
    private RelativeEncoder m_gripperEncoder;
    private PIDController m_positionController;
    private PIDController m_velocityController;
    private BufferZone m_velocityBuffer;

    public Gripper(RelativeEncoder gripperEncoder){

        this.m_motor = new WL_SparkMax(Constants.GripperConstants.kCanCoderID, Constants.GripperConstants.kGripperMotorType); //TODO: find motor id
       
        this.m_motor.getEncoder().setPositionConversionFactor(Constants.GripperConstants.kGripperGearRatio);
        this.m_motor.getEncoder().setVelocityConversionFactor(Constants.GripperConstants.kGripperGearRatio);

        this.m_gripperEncoder = gripperEncoder;

        this.m_velocityController = new PIDController(0, 0, 0);
        this.m_positionController = new PIDController(0, 0, 0); //TODO: Tune
        this.m_positionController.setTolerance(1);
        
        this.m_velocityBuffer = new BufferZone(Constants.GripperConstants.kGripperMinVelocity, Constants.GripperConstants.kGripperMaxVelocity, Constants.GripperConstants.kGripperOpen, Constants.GripperConstants.kGripperClosed,Constants.GripperConstants.kGripperBuffer);
        
        this.m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(true);
        this.m_motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(true);

   
    }
    //* get & at methods */
    public boolean atVelocitySetpoint() {
        return m_velocityController.atSetpoint();
    }
    public boolean atPositionSetpoint(){
        return(m_positionController.atSetpoint());

    }
    public double getEncoderVelocity() {
        return m_motor.getEncoder().getVelocity();
    }
    public double getEncoderPosition(){

        return m_motor.getEncoder().getPosition();
    }
    public boolean getReverseLimitSwitch() {
        return m_motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }

    public boolean getForwardLimitSwitch() {
        return m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }


    //*geneal util methods */
    private void setPWM(double pwm){
        m_motor.set(pwm);

    }
    

    public void resetPIDs() {
        m_velocityController.reset();
        m_positionController.reset();
    }

    //* set and run methods */
    private void runVelocityPID(double velocity) { //private because in the usecase i was provided with, this should never be controlled outside of positional control. Feel free to change.
       
        this.setPWM(m_velocityController.calculate(this.getEncoderVelocity(), m_velocityBuffer.get(velocity, getEncoderPosition())));
        
    }
    private void runUnclampedVelocityPID(double velocity) {
        //        this.setPWM(m_velocityController.calculate(this.getEncoderVelocity(), MathUtil.clamp(velocity, Constants.Hood.HOOD_MIN_VELOCITY, Constants.Hood.HOOD_MAX_VELOCITY)));
                this.setPWM(m_velocityController.calculate(this.getEncoderVelocity(), velocity));
    }
    
    public void runPositionPID(double position) { //public because this is expectecd to be used
        runVelocityPID(m_positionController.calculate(this.getEncoderPosition(), MathUtil.clamp(position, Constants.GripperConstants.kGripperOpen, Constants.GripperConstants.kGripperClosed)));
    }
    public void setEncoderPosition(double position) {
        m_gripperEncoder.setPosition(position);
    
    }
    

    public void periodic(){


    }

}
