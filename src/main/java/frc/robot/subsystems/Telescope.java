// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TelescopeConstants.*;
//import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.robot.Constants;
//import frc.team2485.WarlordsLib.motorcontrol.WL_TalonFX;
//import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Telescope extends SubsystemBase {
  private static final double kTelescopeStartPosition = 0;
  /** Creates a new Telescope. */

  private double feedForwardVoltage = 0;
  private double m_positionSetpointMeters = 0;
  private double m_voltageSetpoint = 0;
  private boolean m_enables = false;
  private boolean m_voltageOverride = false;

  private double m_lastVelocitySetpoint = 0;
  private double outputPercentage; //reconsider naming, vaguely copied over

  private double m_feedbackOutput = 0; //reconsider naming, vaguely copied over
  private double m_feedforwardOutput = 0; //reconsider naming, vaguely copied over

  private WPI_TalonFX m_TelescopeTalon = new WL_TalonFX(kTelescopePort);         

       private final SimpleMotorFeedforward m_feedforward =
        new SimpleMotorFeedforward(
          kSTelescopeVolts, kVTelescopeVoltSecondsPerMeter, kATelescopeVoltSecondsSquaredPerMeter);

  ProfiledPIDController telescopeController = 
    new ProfiledPIDController(
     kPTelescope,
     kITelescope,
     kDTelescope, new TrapezoidProfile.Constraints(5, 10));


  public Telescope() {



    
    public double getDistance (){

        double distance = 1;

        return distance;

    }

    public boolean isInverted (){
      return m_TelescopeTalon.getInverted();
    }

    public void invertTalon(){
      if (m_TelescopeTalon.getInverted()){
        m_TelescopeTalon.setInverted(false);
      }
      else {
        m_TelescopeTalon.setInverted(true);
      }
    }

    public void setLowerNodePositionMeters(double position){
      m_voltageOverride = false;
      m_positionSetpointMeters = MathUtil.clamp(position, kTelescopeStartPosition, kTelescopeLowerPosition);
    }

    public void setMiddleNodePositionMeters(double position){
      m_voltageOverride = false;
      m_positionSetpointMeters = MathUtil.clamp(position, kTelescopeStartPosition, kTelescopeMiddlePosition);
    }

    public void setUpperNodePositionMeters(double position){
      m_voltageOverride = false;
      m_positionSetpointMeters = MathUtil.clamp(position, kTelescopeStartPosition, kTelescopeUpperPosition);
    }

    public double getError () {
      return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
    }
    
    public double getPositionMeters(){
      return m_TelescopeTalon.getSelectedSensorPosition();
    }

    public void resetPositionMeters (double position) {
      m_TelescopeTalon.setSelectedSensorPosition(position);
    }

    public double getVelocityMetersPerSecond (){
      return m_TelescopeTalon.getSelectedSensorVelocity();
    }

    public void setVoltage (double voltage){
      m_voltageOverride = true;
      m_voltageSetpoint = voltage;

    }


    // yuvi's to do list for 2-11-23: resetPositionMeters, getVelocityMetersPerSecond, setVoltage, periodic
    
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_voltageOverride){
      m_TelescopeTalon.set(ControlMode.PercentOutput, m_voltageSetpoint/12); //12 is kNominalVoltage but when I wrote kNominalVoltage it was being mad and bitchy so I just wrote 12, i'm too lazy to go into constants
      

    }
    else {
      double feedbackOutputVoltage = 0;


        feedbackOutputVoltage = 
          telescopeController.calculate(this.getPositionMeters(), m_positionSetpointMeters);
      
      double feedForwardOutputVoltage = 0;


        feedForwardOutputVoltage = 
          m_feedforward.calculate(m_lastVelocitySetpoint, telescopeController.getSetpoint().velocity, kTelescopeControlLoopTimeSeconds);

      outputPercentage = 
        (feedbackOutputVoltage + feedForwardOutputVoltage / 12); //same deal as above  

      m_feedbackOutput = feedbackOutputVoltage;
      m_feedforwardOutput = feedForwardOutputVoltage;

      m_TelescopeTalon.set(ControlMode.PercentOutput, outputPercentage);

      m_lastVelocitySetpoint = telescopeController.getSetpoint().velocity;
    }


    

  }
}
