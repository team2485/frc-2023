// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GamePieceHandling.Gripper;
import frc.robot.subsystems.GamePieceHandling.Gripper.m_pieceType;

import static frc.robot.Constants.GripperConstants;

public class GripperCommands extends CommandBase {
  private final Gripper m_gripper;

  /** Creates a new GripperCommands. */
  public GripperCommands(Gripper m_gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_gripper = m_gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

//   public Command moveArmToClosedSetpoint() {
//     return moveGripperToSetpoint(0);
//   }

//   public Command moveArmToOpenSetpoint() {
//     m_gripper.setCurrentHeldPiece(m_pieceType.Cone);
//     return moveGripperToSetpoint(GripperConstants.kGripperOpenPositionSetpoint);
//   }

//   private Command moveGripperToSetpoint(double gripperControllerSetpoint) {
//     m_gripper.setPositionSetpoint(gripperControllerSetpoint);
//     return new RunCommand(()-> m_gripper.setGripperPower(m_gripper.getPIDControllerOutput())).until(()-> gripperMoveCompleted()).andThen(()-> m_gripper.setGripperPower(0));
//   }

//   public boolean gripperMoveCompleted() {
//     return m_gripper.isStopped() || m_gripper.isAtSetpoint();

//     // This is from the current based approach
//     // return m_gripper.isStalling() || m_gripper.isAtSetpoint();
//   }
}