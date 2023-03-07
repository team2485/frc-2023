// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.GamePieceHandlingCommands;
import frc.robot.commands.auto.AutoCommandBuilder;
import frc.robot.subsystems.GamePieceStateMachine;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.GamePieceHandling.Elevator;
import frc.robot.subsystems.GamePieceHandling.Gripper;
import frc.robot.subsystems.GamePieceHandling.Intake;
import frc.robot.subsystems.GamePieceHandling.IntakeArm;
import frc.robot.subsystems.GamePieceHandling.IntakeServo;
import frc.robot.subsystems.GamePieceHandling.Magazine;
import frc.robot.subsystems.GamePieceHandling.Telescope;
import frc.robot.subsystems.GamePieceHandling.Wrist;
import frc.robot.subsystems.GamePieceHandling.Gripper.m_gripperStates;
import frc.robot.subsystems.GamePieceHandling.Intake.m_intakeStates;
import frc.robot.subsystems.GamePieceHandling.IntakeArm.m_intakeArmStates;
import frc.robot.subsystems.GamePieceHandling.Wrist.m_wristStates;
import frc.robot.subsystems.Vision.m_visionStates;
import frc.robot.subsystems.drive.Drivetrain;
import frc.util.PoseEstimation;
import io.github.oblarg.oblog.annotations.Log;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final WL_CommandXboxController m_driver = new WL_CommandXboxController(OIConstants.kDriverPort);

  private final WL_CommandXboxController m_operator = new WL_CommandXboxController(OIConstants.kOperatorPort);

  public final Drivetrain m_drivetrain = new Drivetrain();
  public final Elevator m_elevator = new Elevator();
  public final Wrist m_wrist = new Wrist();
  public final Gripper m_gripper = new Gripper();
  public final Telescope m_telescope = new Telescope();
  public final IntakeArm m_intakeArm = new IntakeArm();
  public final Intake m_intake = new Intake();
  public final Magazine m_magazine = new Magazine();
  public final IntakeServo m_servo = new IntakeServo();
  public final PhotonCamera m_camera = new PhotonCamera(VisionConstants.kCameraName);
  public final PoseEstimation m_poseEstimator = new PoseEstimation(m_camera, m_drivetrain);
  // public final Vision m_vision = new Vision(m_drivetrain, m_poseEstimator::getCurrentPose);
  public final AlignToTag m_alignToTag = new AlignToTag(m_camera, m_drivetrain, m_poseEstimator::getCurrentPose);

  public GamePieceStateMachine m_stateMachine = new GamePieceStateMachine();

  @Log(name = "Auto Chooser", width = 2, height = 2, rowIndex = 4, columnIndex = 0)
  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_drivetrain.zeroGyro();

    m_autoChooser.setDefaultOption("Test", AutoCommandBuilder.testAuto(m_drivetrain, m_gripper));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    this.configureDrivetrainCommands();
    this.configureGamePieceCommands();
    this.configureVisionCommands();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void configureDrivetrainCommands() {
    m_drivetrain.setDefaultCommand(
        new DriveWithController(m_drivetrain,
            () -> m_driver.getLeftY(),
            () -> m_driver.getLeftX(),
            () -> m_driver.getRightX(),
            () -> {
              return m_driver.rightBumper().getAsBoolean();
            }));

    m_driver.x().onTrue(new InstantCommand(() -> m_drivetrain.zeroGyro()));
    m_driver.y().onTrue(new InstantCommand(m_drivetrain::resetToAbsolute));

    // hypothetical state machine formatting (delete later)
  }

  private void configureGamePieceCommands() {
    // m_operator.rightPOV().onTrue(new
    // InstantCommand(()->m_elevator.requestState(m_elevatorStates.StateMiddleCube)));

    m_driver.rightTrigger()
        .whileTrue(GamePieceHandlingCommands.deployIntakeCommand(m_intakeArm, m_intake, m_magazine, m_telescope,
            m_elevator, m_wrist, m_gripper))
        .onFalse(GamePieceHandlingCommands.retractIntakeCommand(m_intakeArm, m_intake, m_magazine, m_telescope,
            m_elevator, m_wrist));

    m_driver.rightBumper()
        .whileTrue(GamePieceHandlingCommands.outtakeCommand(m_intakeArm, m_intake, m_magazine, m_telescope, m_elevator,
            m_wrist))
        .onFalse(GamePieceHandlingCommands.retractIntakeCommand(m_intakeArm, m_intake, m_magazine, m_telescope,
            m_elevator, m_wrist));

    m_operator.b().onTrue(
        new ConditionalCommand(new InstantCommand(() -> m_gripper.requestState(m_gripperStates.StateInit)),
            new ConditionalCommand(
                GamePieceHandlingCommands.liftFromIntakeCommand(m_elevator, m_wrist, m_gripper),
                new InstantCommand(() -> m_gripper.requestState(m_gripperStates.StateGrip)),
                () -> {
                  return m_wrist.getSetpoint() == 0;
                }),
            () -> {
              return m_gripper.getSetpoint() == 1.5;
            }));

    m_driver.leftBumper().onTrue(
        new ConditionalCommand(new InstantCommand(() -> m_intakeArm.requestState(m_intakeArmStates.StateRetracted)),
            GamePieceHandlingCommands.deployAndLockIntakeCommand(m_intakeArm, m_telescope, m_elevator, m_wrist),
            () -> {
              return IntakeArm.m_intakeArmState == m_intakeArmStates.StateDeployAndLock;
            }));

    m_driver.leftTrigger().onTrue(new InstantCommand(() -> m_intake.requestState(m_intakeStates.StateOn)))
        .onFalse(new InstantCommand(() -> m_intake.requestState(m_intakeStates.StateOff)));

    m_driver.y().whileTrue(new InstantCommand(() -> m_intake.requestState(Intake.m_intakeStates.StateOut)))
        .onFalse(new InstantCommand(() -> m_intake.requestState(m_intakeStates.StateOff)));

    m_operator.a().onTrue(GamePieceHandlingCommands.travelSetpoint(m_telescope, m_elevator, m_gripper, m_wrist));
    m_operator.x().onTrue(new InstantCommand(() -> m_gripper.requestState(m_gripperStates.StateInit)));

    m_operator.lowerPOV().onTrue(GamePieceHandlingCommands.lowSetpoint(m_telescope, m_elevator, m_gripper, m_wrist));

    m_operator.leftPOV().onTrue(
        new ConditionalCommand(GamePieceHandlingCommands.midCubeSetpoint(m_telescope, m_elevator, m_gripper, m_wrist),
            GamePieceHandlingCommands.midConeSetpoint(m_telescope, m_elevator, m_gripper, m_wrist),
            () -> {
              return Gripper.currentPieceType == Gripper.m_pieceType.Cube;
            }));

    m_operator.upperPOV().onTrue(
        new ConditionalCommand(GamePieceHandlingCommands.highCubeSetpoint(m_telescope, m_elevator, m_gripper, m_wrist),
            GamePieceHandlingCommands.highConeSetpoint(m_telescope, m_elevator, m_gripper, m_wrist),
            () -> {
              return Gripper.currentPieceType == Gripper.m_pieceType.Cube;
            }));

    m_operator.rightPOV().onTrue(
        GamePieceHandlingCommands.doubleSubstationSetpoint(m_telescope, m_elevator, m_wrist));

    m_operator.leftTrigger().onTrue(new InstantCommand(() -> m_wrist.requestState(m_wristStates.StateDown)));
  }

  private void configureVisionCommands() {
    // TODO: update bindings for vision offsets

    // m_driver.a().toggleOnTrue(m_alignToTag);

    // m_driver.a().toggleOnTrue(new InstantCommand(() -> m_vision.requestState(m_visionStates.StateAlign)));
    // m_driver.a().toggleOnFalse(new InstantCommand(() -> m_vision.requestState(m_visionStates.StateIdle)));

    // m_driver.leftPOV().onTrue(new InstantCommand(() ->
    // m_alignToTag.addOffset(-Vision.kOffsetToNextScoringStation)));
    // m_driver.rightPOV().onTrue(new InstantCommand(() ->
    // m_alignToTag.addOffset(Vision.kOffsetToNextScoringStation)));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}