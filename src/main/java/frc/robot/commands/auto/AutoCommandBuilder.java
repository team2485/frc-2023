package frc.robot.commands.auto;

import static frc.robot.commands.auto.PathCommandBuilder.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.GamePieceHandling.Gripper;
import frc.robot.subsystems.GamePieceHandling.Gripper.m_gripperStates;
import frc.robot.subsystems.drive.Drivetrain;


public class AutoCommandBuilder {

  public static Command testAuto(Drivetrain drivetrain, Gripper gripper) {

    WL_SwerveControllerCommand path = getPathCommand(drivetrain, "BlueLeft2Piece");

    return new WaitCommand(5.5).andThen(getResetOdometryCommand(drivetrain, path), path, new WaitCommand(2), new InstantCommand(()->Gripper.requestState(m_gripperStates.StateGrip)));
    }
}
  