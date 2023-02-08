package frc.robot.commands;

import static frc.robot.Constants.MagazineConstants.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Magazine;
import frc.robot.subsystems.drive.magazine;
import frc.robot.subsystems.drive.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CargoHandlingCommandBuilder {

    public static Command magazineOff(Magazine magazine) {
        return new InstantCommand(() -> magazine.setVelocityRotationsPerSecond(0));
      }

      public static Command magazineOn(magazine magazine) {
        return new InstantCommand(() -> magazine.setVelocityRotationsPerSecond(5));
      }
  
      
}