package frc.robot.commands;

import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;

public class CargoHandlingCommandBuilder {

  public static Command setWristBottom(Wrist wrist) {
    return new InstantCommand(() -> wrist.setAngleRadians(kWristBottomPositionRadians));
  }

  public static Command setWristMP(Wrist wrist) {
    return new InstantCommand(
        () -> wrist.setAngleRadians((kWristBottomPositionRadians + kWristTopPositionRadians) / 2));
  }

}
