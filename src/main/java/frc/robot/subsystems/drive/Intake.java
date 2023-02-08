// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;


import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;





public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kIntakeTalonPort);

  private final SR_SimpleMotorFeedforward m_feedforward = 
                                  new SR_SimpleMotorFeedforward(kSIntakeVolts, kVIntakeVoltSecondsPerMeter, kAIntakeVoltSecondsSquaredPerMeter);

  @Log(name = "Velocity Setpoint")
  private double m_velocitySetpointRotationsPerSecond;

  private double m_lastVelocitySetpoint;

  @Log(name = "Feedforward output")
  private double m_feedforwardOutput;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  @Log(name = "output voltage")
  private double m_lastOutputVoltage = 0;

  public Intake(){
    m_talon.configVoltageCompSaturation(Constants.kNominalVoltage);
    m_talon.enableVoltageCompensation(true);
    m_talon.setNeutralMode(NeutralMode.Brake);
    //m_talon.setInverted(true);
    //maybe add current limiting questionmark
  }


  @Log(name = "Current Velocity (RPS)")
  public double getVelocityRotationsPerSecond(){
    return m_talon.getSelectedSensorVelocity() / (kIntakeGearRatio *  Constants.kFalconSensorUnitsPerRotation * 10);
  }

  @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond){
    m_voltageOverride = false;

    m_velocitySetpointRotationsPerSecond = rotationsPerSecond;
  }

  @Config.NumberSlider(name = "setVoltage", min = -12, max = 12)
  public void setVoltage(double voltage){
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }


  public void runControlLoop(){
    double outputVoltage = 0;
    if(m_voltageOverride){
      outputVoltage = m_voltageSetpoint;
    }
    else{
      double feedForwardOutput = m_feedforward.calculate(m_lastVelocitySetpoint, m_velocitySetpointRotationsPerSecond, kIntakeLoopTimeSeconds);

      outputVoltage = feedForwardOutput;
      m_feedforwardOutput = feedForwardOutput;
    }

    if(outputVoltage != m_lastOutputVoltage){
      m_talon.setVoltage(outputVoltage);
    }

    m_lastOutputVoltage = outputVoltage;
  }

  @Override
  public void periodic(){
    this.runControlLoop();
  }
}  
