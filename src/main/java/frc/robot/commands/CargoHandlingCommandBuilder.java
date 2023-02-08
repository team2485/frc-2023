package frc.robot.commands;

import static frc.robot.Constants.MagazineConstants.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Wrist;
import frc.robot.subsystems.drive.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CargoHandlingCommandBuilder {

    public static Command setWristBottom(Wrist wrist) {
        return new InstantCommand(() -> wrist.setAngleRadians(kWristBottomPositionRadians));
      }

      public static Command setWristMP(Wrist wrist) {
        return new InstantCommand(() -> wrist.setAngleRadians((kWristBottomPositionRadians+kWristTopPositionRadians)/2));
      }
  
      
}
