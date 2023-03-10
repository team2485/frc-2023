package frc.robot.commands.auto;

import static frc.robot.commands.auto.PathCommandBuilder.*;

import java.time.Instant;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.GamePieceHandling.Elevator;
import frc.robot.subsystems.GamePieceHandling.Gripper;
import frc.robot.subsystems.GamePieceHandling.Intake;
import frc.robot.subsystems.GamePieceHandling.IntakeArm;
import frc.robot.subsystems.GamePieceHandling.Telescope;
import frc.robot.subsystems.GamePieceHandling.Wrist;
import frc.robot.subsystems.GamePieceHandling.Elevator.m_elevatorStates;
import frc.robot.subsystems.GamePieceHandling.Gripper.m_gripperStates;
import frc.robot.subsystems.GamePieceHandling.Intake.m_intakeStates;
import frc.robot.subsystems.GamePieceHandling.IntakeArm.m_intakeArmStates;
import frc.robot.subsystems.GamePieceHandling.Telescope.m_telescopeStates;
import frc.robot.subsystems.GamePieceHandling.Wrist.m_wristStates;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.Drivetrain;


public class AutoCommandBuilder {

  
   public static Command onePieceClimb(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope, IntakeArm intakeArm){
    WL_SwerveControllerCommand path = getPathSlowCommand(drivetrain, "Middle1PieceClimb");

    return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.3), new InstantCommand(()->intakeArm.requestState(m_intakeArmStates.StateDeployAndLock)), new WaitCommand(1),
        getResetOdometryCommand(drivetrain, path), 
        path.withTimeout(2),new AutoBalance(drivetrain));
   }

   public static Command twoPieceBlue(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope ) {

    WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Blue2PiecePt1");
    WL_SwerveControllerCommand path2 = getPathCommand(drivetrain, "Blue2PiecePt2");


    return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.3), getResetOdometryCommand(drivetrain, path), 
        path.alongWith(new WaitCommand(3.2).andThen(new InstantCommand(()->Gripper.requestState(m_gripperStates.StateAutoGrip)))).withTimeout(3.2),
        new InstantCommand(()->drivetrain.drive(new Translation2d(0,0), 0, true, true)),
        new WaitCommand(0.25), path2.withTimeout(3.5));
        
    }

    public static Command onePieceBlue(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope){
        WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Blue1Piece");

        return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.3),getResetOdometryCommand(drivetrain, path), 
        path.withTimeout(4));
    }


    
   public static Command twoPieceRed(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope ) {

    WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Red2PiecePt1");
    WL_SwerveControllerCommand path2 = getPathCommand(drivetrain, "Red2PiecePt2");


    return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.3), getResetOdometryCommand(drivetrain, path), 
        path.alongWith(new WaitCommand(3.2).andThen(new InstantCommand(()->Gripper.requestState(m_gripperStates.StateAutoGrip)))).withTimeout(3.2),
        new InstantCommand(()->drivetrain.drive(new Translation2d(0,0), 0, true, true)),
        new WaitCommand(0.25), path2.withTimeout(3.5));
        
    }

    public static Command onePieceRed(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope){
        WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Red1Piece");
        
        return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.3),getResetOdometryCommand(drivetrain, path), 
        path.withTimeout(4));
    }


    public static Command autoInit(Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope){
      return new InstantCommand(()->Elevator.m_elevatorState=m_elevatorStates.StateAutoWait)
          .andThen(new InstantCommand(()->Telescope.m_telescopeState=m_telescopeStates.StateAutoWait),
                  new InstantCommand(()->Wrist.m_wristState=m_wristStates.StateAutoWait),
                  new InstantCommand(()->Gripper.m_gripperState=m_gripperStates.StateAutoWait),
                  new InstantCommand(()->Intake.m_intakeState=m_intakeStates.StateAutoInit));
  }

  public static Command test(Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope){
    return new InstantCommand(()->Elevator.m_elevatorState=m_elevatorStates.StateFault)
        .andThen(new InstantCommand(()->Telescope.m_telescopeState=m_telescopeStates.StateFault),
                new InstantCommand(()->Wrist.m_wristState=m_wristStates.StateFault),
                new InstantCommand(()->Gripper.m_gripperState=m_gripperStates.StateFault));
}
  
}
