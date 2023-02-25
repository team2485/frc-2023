// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GamePieceHandling.Elevator;
import frc.robot.subsystems.GamePieceHandling.Gripper;
import frc.robot.subsystems.GamePieceHandling.Telescope;
import frc.robot.subsystems.GamePieceHandling.Wrist;
import frc.robot.subsystems.GamePieceHandling.Elevator.m_elevatorStates;
import frc.robot.subsystems.GamePieceHandling.Gripper.m_gripperStates;
import frc.robot.subsystems.GamePieceHandling.Telescope.m_telescopeStates;
import frc.robot.subsystems.GamePieceHandling.Wrist.m_wristStates;

/** Add your docs here. */
public class GamePieceHandlingCommands {

    public static Command lowSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist){
        ParallelCommandGroup group = new ParallelCommandGroup(new RunCommand(()->wrist.requestState(m_wristStates.StateMiddle)),
            new RunCommand(()->telescope.requestState(m_telescopeStates.StateIn)),
                       new RunCommand(()->elevator.requestState(m_elevatorStates.StateBottom)));

        return group.until(()->telescope.atSetpoint() && elevator.atSetpoint() && wrist.atSetpoint())
                            .andThen(new InstantCommand(()->gripper.requestState(m_gripperStates.StateInit)));
    }

}
