package frc.robot.commands.auto;

import static frc.robot.commands.auto.PathCommandBuilder.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.GamePieceHandling.Elevator;
import frc.robot.subsystems.GamePieceHandling.Gripper;
import frc.robot.subsystems.GamePieceHandling.Telescope;
import frc.robot.subsystems.GamePieceHandling.Wrist;
import frc.robot.subsystems.GamePieceHandling.Elevator.m_elevatorStates;
import frc.robot.subsystems.GamePieceHandling.Gripper.m_gripperStates;
import frc.robot.subsystems.GamePieceHandling.Telescope.m_telescopeStates;
import frc.robot.subsystems.GamePieceHandling.Wrist.m_wristStates;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.Drivetrain;


public class AutoCommandBuilder {

   public static Command testAuto(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope ) {

    WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Test");

    return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(6), path);
    }


    public static Command autoInit(Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope){
      return new InstantCommand(()->Elevator.m_elevatorState=m_elevatorStates.StateAutoWait)
          .andThen(new InstantCommand(()->Telescope.m_telescopeState=m_telescopeStates.StateAutoWait),
                  new InstantCommand(()->Wrist.m_wristState=m_wristStates.StateAutoWait),
                  new InstantCommand(()->Gripper.m_gripperState=m_gripperStates.StateAutoWait));
  }
  
}
