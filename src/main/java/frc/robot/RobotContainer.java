// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.GamePieceHandling;
import frc.robot.commands.auto.AutoCommandBuilder;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.Intake;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final WL_CommandXboxController m_driver = new WL_CommandXboxController(OIConstants.kDriverPort);

  private final WL_CommandXboxController m_operator = new WL_CommandXboxController(OIConstants.kOperatorPort);

  private final Intake m_intake = new Intake();

  public final Drivetrain m_drivetrain;

  @Log(name = "Auto Chooser", width = 2, height = 2, rowIndex = 4, columnIndex = 0)
  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_drivetrain = new Drivetrain();
    m_drivetrain.zeroGyro();
    configureBindings();

    m_autoChooser.setDefaultOption("Test", AutoCommandBuilder.testAuto(m_drivetrain));
  }

  /** 
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    configureDrivetrainCommands();
    configureGamePieceHandlingCommands();

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void configureDrivetrainCommands(){

    m_drivetrain.setDefaultCommand(
        new DriveWithController(m_drivetrain, 
                                ()->m_driver.getLeftY(), 
                                ()->m_driver.getLeftX(), 
                                ()->m_driver.getRightX(), 
                                ()->{return m_driver.rightBumper().getAsBoolean();})
    );

    m_driver.x().onTrue(new InstantCommand(()->m_drivetrain.zeroGyro()));

    m_driver.y().onTrue(new InstantCommand(m_drivetrain::resetToAbsolute));
  }

  private void configureGamePieceHandlingCommands(){
    m_driver.rightTrigger().whileTrue(GamePieceHandling.getRunIntakeCommand(m_intake))
                           .whileFalse(GamePieceHandling.getStopIntakeCommand(m_intake));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}


