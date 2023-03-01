// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        ParallelCommandGroup group = new ParallelCommandGroup(new InstantCommand(()->wrist.requestState(m_wristStates.StateBottom)),
            new InstantCommand(()->telescope.requestState(m_telescopeStates.StateIn)),
                       new InstantCommand(()->elevator.requestState(m_elevatorStates.StateBottom)));
        return group;
    }

    public static Command midCubeSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist){
        ParallelCommandGroup group = new ParallelCommandGroup(new InstantCommand(()->wrist.requestState(m_wristStates.StateMiddle)),
            new InstantCommand(()->telescope.requestState(m_telescopeStates.StateMiddleCube)),
                       new InstantCommand(()->elevator.requestState(m_elevatorStates.StateMiddleCube)));
        return group;
    }

    public static Command highCubeSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist){
        ParallelCommandGroup group = new ParallelCommandGroup(new InstantCommand(()->wrist.requestState(m_wristStates.StateMiddle)),
            new InstantCommand(()->telescope.requestState(m_telescopeStates.StateOutCube)),
                       new InstantCommand(()->elevator.requestState(m_elevatorStates.StateTopCube)));

        return group;
    }

    public static Command midConeSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist){
        ParallelCommandGroup group = new ParallelCommandGroup(new InstantCommand(()->wrist.requestState(m_wristStates.StateTop)),
            new InstantCommand(()->telescope.requestState(m_telescopeStates.StateMiddleCone)),
                       new InstantCommand(()->elevator.requestState(m_elevatorStates.StateMiddleCone)));

        return group;
    }
    public static Command highConeSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist){
        ParallelCommandGroup group = new ParallelCommandGroup(new InstantCommand(()->wrist.requestState(m_wristStates.StateHigh)),
            new InstantCommand(()->telescope.requestState(m_telescopeStates.StateOutCone)),
                       new InstantCommand(()->elevator.requestState(m_elevatorStates.StateTopCone)));

        return group;
    }

    public static Command travelSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist){
        ParallelCommandGroup group = new ParallelCommandGroup(new InstantCommand(()->wrist.requestState(m_wristStates.StateMiddle)),
        new InstantCommand(()->telescope.requestState(m_telescopeStates.StateIn)),
                   new InstantCommand(()->elevator.requestState(m_elevatorStates.StateLow)));

    return group;
}

    public static Command doubleSubstationSetpoint(Telescope telescope, Elevator elevator, Wrist wrist){
        ParallelCommandGroup group = new ParallelCommandGroup(new InstantCommand(()->wrist.requestState(m_wristStates.StateMiddle)),
        new InstantCommand(()->telescope.requestState(m_telescopeStates.StateMiddleCone)),
                   new InstantCommand(()->elevator.requestState(m_elevatorStates.StateTopCone)));

    return group;
    }

}
