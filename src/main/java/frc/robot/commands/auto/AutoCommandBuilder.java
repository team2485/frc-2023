package frc.robot.commands.auto;

import static frc.robot.commands.auto.PathCommandBuilder.*;

import java.lang.constant.DirectMethodHandleDesc;

import javax.management.InstanceAlreadyExistsException;
import javax.swing.plaf.metal.MetalIconFactory.TreeLeafIcon;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.PoseEstimation;
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
import frc.robot.subsystems.drive.Drivetrain;

public class AutoCommandBuilder {
        private Drivetrain m_drivetrain;
        private final PoseEstimation m_poseEstimator;

        public AutoCommandBuilder(Drivetrain drivetrain, PoseEstimation poseEstimator) {
                this.m_drivetrain = drivetrain;
                this.m_poseEstimator = poseEstimator;
        }

        public static Command onePieceClimb(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist,
                        Telescope telescope, IntakeArm intakeArm) {
                WL_SwerveControllerCommand path1 = getPathSlowCommand(drivetrain, "Middle1PieceClimbPt1");
                WL_SwerveControllerCommand path2 = getPathSlowCommand(drivetrain, "Middle1PieceClimbPt2");


                return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.3),
                                // new InstantCommand(() -> intakeArm.requestState(m_intakeArmStates.StateDeployAndLock)),
                                // new WaitCommand(0.5),
                                getResetOdometryCommand(drivetrain, path1),
                                path1.alongWith(new WaitCommand(1.5).andThen(new InstantCommand(()->Wrist.requestState(m_wristStates.StateVeryTop)))).withTimeout(4),
                                new InstantCommand(()->drivetrain.drive(new Translation2d(0,0), 0, false, false)),new WaitCommand(0.75),
                                path2.withTimeout(1.75),
                                new AutoBalance(drivetrain).withTimeout(8.5), new InstantCommand(drivetrain::autoGyro));
        }

        public Command twoPieceBlue(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist,
                        Telescope telescope) {

                WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Blue2PiecePt1");
                WL_SwerveControllerCommand path2 = getPathCommand(drivetrain, "Blue2PiecePt2");

                return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.0),
                                getResetOdometryCommand(drivetrain, path),
                                path.alongWith(new WaitCommand(3.0)
                                                .andThen(new InstantCommand(() -> Gripper
                                                                .requestState(m_gripperStates.StateAutoGrip))))
                                                .withTimeout(3.5),
                                new InstantCommand(() -> drivetrain.drive(new Translation2d(0, 0), 0, true, true)),
                                new WaitCommand(0.5), path2.withTimeout(2.75), driveToPose(true, false, true, true).withTimeout(2.5), new InstantCommand(drivetrain::autoGyro));

        }

        public static Command twoPieceBlueBottom(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist,
                        Telescope telescope) {
                WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Blue2PieceBottom");

                return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.3),
                getResetOdometryCommand(drivetrain, path), new InstantCommand(()->IntakeArm.requestState(m_intakeArmStates.StateAutoWait)),
                path.withTimeout(6.75), new InstantCommand(()->IntakeArm.requestState(m_intakeArmStates.AutoStateOuttake)), new InstantCommand(drivetrain::zeroGyro));
        }
        
        //anticipatory collision detection paths  

        public Command twoPieceRed(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist,
                        Telescope telescope) {

                WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Red2PiecePt1");   
                WL_SwerveControllerCommand path2 = getPathCommand(drivetrain, "Red2PiecePt2");

                return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.3),
                                getResetOdometryCommand(drivetrain, path),
                                path.alongWith(new WaitCommand(3.2)
                                                .andThen(new InstantCommand(() -> Gripper
                                                                .requestState(m_gripperStates.StateAutoGrip))))
                                                .withTimeout(3.2),      
                                new InstantCommand(() -> drivetrain.drive(new Translation2d(0, 0), 0, true, true)),
                                new WaitCommand(0.5), path2.withTimeout(2.75), driveToPose(false, false, true, true).withTimeout(2.5), new InstantCommand(drivetrain::autoGyro));

        }

        public static Command twoPieceRedBottom(Drivetrain drivetrain, Elevator elevator, Gripper gripper, Wrist wrist,
                        Telescope telescope) {
                WL_SwerveControllerCommand path = getPathCommand(drivetrain, "Red2PieceBottom");

                return autoInit(elevator, gripper, wrist, telescope).andThen(new WaitCommand(4.3),
                                getResetOdometryCommand(drivetrain, path), new InstantCommand(()->IntakeArm.requestState(m_intakeArmStates.StateAutoWait)),
                                path.withTimeout(6.75), new InstantCommand(()->IntakeArm.requestState(m_intakeArmStates.AutoStateOuttake)), new InstantCommand(drivetrain::zeroGyro));
        }

        public Command driveToPose(boolean left, boolean middle, boolean useAllianceColor, boolean inAuto) {
                return new DriveToPose(m_drivetrain, m_poseEstimator::getCurrentPose, left, middle, useAllianceColor, inAuto);
        }

        public Command driveToPose(boolean left, boolean middle, boolean inAuto, TrapezoidProfile.Constraints xyConstraints,
                        TrapezoidProfile.Constraints omegConstraints, boolean useAllianceColor) {
                return new DriveToPose(m_drivetrain, m_poseEstimator::getCurrentPose, left, middle, inAuto, xyConstraints,
                                omegConstraints, useAllianceColor);
        }

        public static Command autoInit(Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope) {
                return new InstantCommand(() -> Elevator.m_elevatorState = m_elevatorStates.StateAutoWait)
                                .andThen(new InstantCommand(
                                                () -> Telescope.m_telescopeState = m_telescopeStates.StateAutoWait),
                                                new InstantCommand(
                                                                () -> Wrist.m_wristState = m_wristStates.StateAutoWait),
                                                new InstantCommand(
                                                                () -> Gripper.m_gripperState = m_gripperStates.StateAutoWait),
                                                new InstantCommand(
                                                                () -> Intake.m_intakeState = m_intakeStates.StateAutoInit));
        }

        // public static Command teleopInit(Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope) {
        //         return new InstantCommand(() -> Elevator.m_elevatorState = m_elevatorStates.StateFault)
        //                 .andThen(new InstantCommand(
        //                         () -> Telescope.m_telescopeState = m_telescopeStates.StateFault),
        //                         new InstantCommand(
        //                                 () -> Wrist.m_wristState = m_wristStates.StateFault),
        //                         new InstantCommand(
        //                                 () -> Gripper.m_gripperState = m_gripperStates.StateFault),
        //                         new InstantCommand(
        //                                 () -> Intake.m_intakeState = m_intakeStates.StateFault));

                                                       
        // }

        public static Command test(Elevator elevator, Gripper gripper, Wrist wrist, Telescope telescope) {
                return new InstantCommand(() -> Elevator.m_elevatorState = m_elevatorStates.StateFault)
                                .andThen(new InstantCommand(
                                                () -> Telescope.m_telescopeState = m_telescopeStates.StateFault),
                                                new InstantCommand(() -> Wrist.m_wristState = m_wristStates.StateFault),
                                                new InstantCommand(
                                                                () -> Gripper.m_gripperState = m_gripperStates.StateFault));
        }

}       
