// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class AlignToPole extends CommandBase implements Loggable{
  /** Creates a new AlignToPole. */

  Drivetrain m_drive;
  PhotonCamera camera;

  @Log(name="has target?", tabName = "RobotContainer")
  boolean hasTarget;

  @Log(name="offset", tabName = "RobotContainer")
  double offset;

  public AlignToPole(Drivetrain drive) {
    m_drive = drive;
    camera = new PhotonCamera("photonvision");
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double kp = -0.0375;
    offset = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("photonvision").getEntry("targetYaw").getDouble(0)-14;
    m_drive.drive(new Translation2d(0, kp*offset), 0, true, true);
}

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(new Translation2d(0,0),0,true,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
