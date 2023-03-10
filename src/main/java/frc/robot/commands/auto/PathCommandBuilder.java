package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.Swerve.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drivetrain;
import frc.WarlordsLib.sendableRichness.SR_PIDController;
import frc.WarlordsLib.sendableRichness.SR_ProfiledPIDController;

public class PathCommandBuilder {
  public static WL_SwerveControllerCommand getPathCommand(Drivetrain drivetrain, String name) {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            name, kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

    // put trajectory on Glass's Field2d widget

    // create controller for robot angle

    // create command to follow path
    var thetaController = new SR_ProfiledPIDController(kPAutoThetaController, kIAutoThetaController, kDAutoThetaController, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    WL_SwerveControllerCommand pathCommand =
        new WL_SwerveControllerCommand(
            path,
            drivetrain::getPose,
            swerveKinematics,
            new SR_PIDController(kPAutoXController, kIAutoXController, kDAutoXController),
            new SR_PIDController(kPAutoYController, kIAutoYController, kDAutoYController),
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

      return pathCommand; 
          }

          public static WL_SwerveControllerCommand getPathSlowCommand(Drivetrain drivetrain, String name) {
            PathPlannerTrajectory path =
                PathPlanner.loadPath(
                    name, kSlowerSpeedMetersPerSecond, kSlowerAccelerationMetersPerSecondSquared);
        
            // put trajectory on Glass's Field2d widget
        
            // create controller for robot angle
        
            // create command to follow path
            var thetaController = new SR_ProfiledPIDController(kPAutoThetaController, kIAutoThetaController, kDAutoThetaController, kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
            WL_SwerveControllerCommand pathCommand =
                new WL_SwerveControllerCommand(
                    path,
                    drivetrain::getPose,
                    swerveKinematics,
                    new SR_PIDController(kPAutoXController, kIAutoXController, kDAutoXController),
                    new SR_PIDController(kPAutoYController, kIAutoYController, kDAutoYController),
                    thetaController,
                    drivetrain::setModuleStates,
                    drivetrain);
        
              return pathCommand; 
                  }

  public static InstantCommand getResetOdometryCommand(
            Drivetrain drivetrain, WL_SwerveControllerCommand pathCommand) {
          return new InstantCommand(
              () ->
                  drivetrain.resetOdometry(
                      new Pose2d(
                          pathCommand.m_trajectory.getInitialState().poseMeters.getTranslation(),
                          pathCommand.m_trajectory.getInitialState().holonomicRotation)),
              drivetrain);
            }

  public static InstantCommand getStopPathCommand(Drivetrain drivetrain) {
    return new InstantCommand(() -> drivetrain.drive(new Translation2d(0,0), 0, false, false));
  }


}
