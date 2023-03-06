package frc.robot.commands.auto;

import static frc.robot.Constants.*;
import static frc.robot.commands.auto.PathCommandBuilder.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.GamePieceHandling.Gripper;
import frc.robot.subsystems.GamePieceHandling.Gripper.m_gripperStates;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.Drivetrain;


public class AutoCommandBuilder {

  public static Command testAuto(Drivetrain drivetrain, Gripper gripper) {

    WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Test");

    return new WaitCommand(5);
    }
}
  