// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;


import static frc.robot.Constants.IntakeConstants.*;


import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;





public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}
  private final WL_SparkMax m_spark = new WL_SparkMax(kIntakeSparkPort);

  private final SR_SimpleMotorFeedforward m_feedforward = 
                                  new SR_SimpleMotorFeedforward(kSIntakeVolts, kVIntakeVoltSecondsPerMeter, kAIntakeVoltSecondsSquaredPerMeter);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
