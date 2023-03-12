// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GamePieceHandling.Elevator;
import frc.robot.subsystems.GamePieceHandling.Gripper;
import frc.robot.subsystems.GamePieceHandling.Intake;
import frc.robot.subsystems.GamePieceHandling.IntakeArm;
import frc.robot.subsystems.GamePieceHandling.Magazine;
import frc.robot.subsystems.GamePieceHandling.Telescope;
import frc.robot.subsystems.GamePieceHandling.Wrist;
import frc.robot.subsystems.GamePieceHandling.Elevator.m_elevatorStates;
import frc.robot.subsystems.GamePieceHandling.Gripper.m_gripperStates;
import frc.robot.subsystems.GamePieceHandling.Intake.m_intakeStates;
import frc.robot.subsystems.GamePieceHandling.IntakeArm.m_intakeArmStates;
import frc.robot.subsystems.GamePieceHandling.Magazine.m_magazineStates;
import frc.robot.subsystems.GamePieceHandling.Telescope.m_telescopeStates;
import frc.robot.subsystems.GamePieceHandling.Wrist.m_wristStates;

/** Add your docs here. */
public class GamePieceHandlingCommands {

  public static Command lowSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> wrist.requestState(m_wristStates.StateBottom)),
        new InstantCommand(() -> telescope.requestState(m_telescopeStates.StateIn)),
        new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateBottom)));
    return group;
  }

  public static Command midCubeSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> wrist.requestState(m_wristStates.StateMiddle)),
        new InstantCommand(() -> telescope.requestState(m_telescopeStates.StateMiddleCube)),
        new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateMiddleCube)));
    return group;
  }

  public static Command highCubeSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> wrist.requestState(m_wristStates.StateMiddle)),
        new InstantCommand(() -> telescope.requestState(m_telescopeStates.StateOutCube)),
        new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateTopCube)));

    return group;
  }

  public static Command midConeSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> wrist.requestState(m_wristStates.StateTop)),
        new InstantCommand(() -> telescope.requestState(m_telescopeStates.StateMiddleCone)),
        new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateMiddleCone)));

    return group;
  }

    public static Command deployMagIntakeCommand(IntakeArm intakeArm, Intake intake, Magazine magazine, Telescope telescope, Elevator elevator, Wrist wrist, Gripper gripper){
        ParallelCommandGroup group = new ParallelCommandGroup(
            new InstantCommand(()->wrist.requestState(m_wristStates.StateDown)),
            new InstantCommand(()->telescope.requestState(m_telescopeStates.StatePickup)),
            new InstantCommand(()->elevator.requestState(m_elevatorStates.StateLow)),
            new InstantCommand(()->intakeArm.requestState(m_intakeArmStates.StateDeployed)),
            new InstantCommand(()->intake.requestState(m_intakeStates.StateOn)),
            new InstantCommand(()->magazine.requestState(m_magazineStates.StateOn)),
            new InstantCommand(()->gripper.requestState(m_gripperStates.StateGrip)));
        
        return group;
    }


  public static Command highConeSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> wrist.requestState(m_wristStates.StateHigh)),
        new InstantCommand(() -> telescope.requestState(m_telescopeStates.StateOutCone)),
        new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateTopCone)));

    return group;
  }

  public static Command travelSetpoint(Telescope telescope, Elevator elevator, Gripper gripper, Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> wrist.requestState(m_wristStates.StateMiddle)),
        new InstantCommand(() -> telescope.requestState(m_telescopeStates.StateIn)),
        new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateTravel)));

    return group;
  }

  public static Command doubleSubstationSetpoint(Telescope telescope, Elevator elevator, Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> wrist.requestState(m_wristStates.StateMiddle)),
        new InstantCommand(() -> telescope.requestState(m_telescopeStates.StateMiddleCone)),
        new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateTopCone)));

    return group;
  }

  public static Command deployIntakeCommand(IntakeArm intakeArm, Intake intake, Magazine magazine, Telescope telescope,
      Elevator elevator, Wrist wrist, Gripper gripper) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> wrist.requestState(m_wristStates.StateDown)),
        new InstantCommand(() -> telescope.requestState(m_telescopeStates.StatePickup)),
        new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateLow)),
        new InstantCommand(() -> intakeArm.requestState(m_intakeArmStates.StateDeployed)),
        new InstantCommand(() -> intake.requestState(m_intakeStates.StateOn)),
        new InstantCommand(() -> magazine.requestState(m_magazineStates.StateOn)),
        new InstantCommand(() -> gripper.requestState(m_gripperStates.StateInit)));

    return group;
  }

  public static Command deployAndLockIntakeCommand(IntakeArm intakeArm, Telescope telescope, Elevator elevator,
      Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> wrist.requestState(m_wristStates.StateDown)),
        new InstantCommand(() -> telescope.requestState(m_telescopeStates.StatePickup)),
        new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateLow)),
        new InstantCommand(() -> intakeArm.requestState(m_intakeArmStates.StateDeployAndLock)));

    return group;

  }

  public static Command outtakeCommand(IntakeArm intakeArm, Intake intake, Magazine magazine, Telescope telescope,
      Elevator elevator, Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> intakeArm.requestState(m_intakeArmStates.StateDeployed)),
        new InstantCommand(() -> intake.requestState(m_intakeStates.StateOut)),
        new InstantCommand(() -> magazine.requestState(m_magazineStates.StateOut)));

    return group;
  }

  public static Command retractIntakeCommand(IntakeArm intakeArm, Intake intake, Magazine magazine, Telescope telescope,
      Elevator elevator, Wrist wrist) {
    ParallelCommandGroup group = new ParallelCommandGroup(
        new InstantCommand(() -> intakeArm.requestState(m_intakeArmStates.StateRetracted)),
        new InstantCommand(() -> intake.requestState(m_intakeStates.StateOff)),
        new InstantCommand(() -> magazine.requestState(m_magazineStates.StateOff)));

    return group;
  }

  public static Command liftFromIntakeCommand(Elevator elevator, Wrist wrist, Gripper gripper) {

    return new InstantCommand(() -> gripper.requestState(m_gripperStates.StateGrip))
        .andThen(new WaitCommand(1),
            new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateMiddleCube)),
            new WaitCommand(0.25),
            new InstantCommand(() -> wrist.requestState(m_wristStates.StateMiddle)));
  }

  public static Command robotRevealll(Elevator elevator, Telescope telescope, Gripper gripper, Wrist wrist) {
    return new InstantCommand(() -> elevator.requestState(m_elevatorStates.StateMiddleCube))
        .andThen(new InstantCommand(() -> gripper.requestState(m_gripperStates.StateGrip)),
            new InstantCommand(() -> wrist.requestState(m_wristStates.StateMiddle)), new WaitCommand(1),
            new InstantCommand(() -> telescope.requestState(m_telescopeStates.StateOutCube)),
            new WaitCommand(1),
            new InstantCommand(() -> gripper.requestState(m_gripperStates.StateInit)),
            new WaitCommand(0.5),
            new InstantCommand(() -> gripper.requestState(m_gripperStates.StateGrip)),
            new WaitCommand(0.5),
            new InstantCommand(() -> gripper.requestState(m_gripperStates.StateInit)),
            new WaitCommand(0.5),
            new InstantCommand(() -> gripper.requestState(m_gripperStates.StateGrip)),
            new WaitCommand(0.5),
            new InstantCommand(() -> gripper.requestState(m_gripperStates.StateInit)),
            new WaitCommand(0.5),
            new InstantCommand(() -> gripper.requestState(m_gripperStates.StateGrip)));
  }
}
