package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Magazine;

public class CargoHandlingCommandBuilder {

  public static Command magazineOff(Magazine magazine) {
    return new InstantCommand(() -> magazine.setVelocityRotationsPerSecond(0));
  }

  public static Command magazineOn(Magazine magazine) {
    return new InstantCommand(() -> magazine.setVelocityRotationsPerSecond(5));
  }

}
