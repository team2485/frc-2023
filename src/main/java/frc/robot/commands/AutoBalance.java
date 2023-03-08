// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  Drivetrain m_drive;
  public double pitch;


  public AutoBalance(Drivetrain drive) {
    m_drive = drive;
    addRequirements(m_drive);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {     
    pitch = m_drive.getPitch();
    double kp = 0.0375;
    m_drive.drive(new Translation2d(pitch*kp, 0), 0, false, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_drive.drive(new Translation2d(0,0),0,false,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
