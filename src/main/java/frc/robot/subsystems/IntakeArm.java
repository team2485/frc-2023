// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeArmConstants;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {
  /** Creates a new IntakeArm. */
  private WPI_TalonFX m_talon = new WPI_TalonFX(IntakeArmConstants.kIntakeArmPort);
  private final SR_ProfiledPIDController m_controller = new SR_ProfiledPIDController(IntakeArmConstants.kPIntakeArm, IntakeArmConstants.kIIntakeArm, IntakeArmConstants.kDIntakeArm,
      IntakeArmConstants.kMotionProfileConstraints);

  public IntakeArm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
