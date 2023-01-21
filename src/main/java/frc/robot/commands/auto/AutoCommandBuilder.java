package frc.robot.commands.auto;

import static frc.robot.Constants.*;
import static frc.robot.commands.auto.PathCommandBuilder.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.Drivetrain;


public class AutoCommandBuilder {

  public static Command testAuto(Drivetrain drivetrain) {

    WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Test");

    return getResetOdometryCommand(drivetrain, path).andThen(path);
              
    }
}
