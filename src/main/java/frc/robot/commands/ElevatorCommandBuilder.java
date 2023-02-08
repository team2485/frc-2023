package frc.robot.commands;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.subsystems.drive.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class ElevatorCommandBuilder extends CommandBase {

    public static Command setPositionMeters(Elevator elevator, double position){
        return new InstantCommand(() -> elevator.setPositionMeters(position));
    }
    public static Command zeroPositionMeters(Elevator elevator){
        return new InstantCommand(() -> elevator.setPositionMeters(0));
    }
}