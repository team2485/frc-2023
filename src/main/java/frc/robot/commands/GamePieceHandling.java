// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Intake;


public class GamePieceHandling {

  public static Command getRunIntakeCommand(Intake intake){
    return new InstantCommand(() -> intake.setVelocityRotationsPerSecond(6), intake);
  }

  public static Command getStopIntakeCommand(Intake intake){
    return new InstantCommand(() -> intake.setVelocityRotationsPerSecond(0));
  }
}