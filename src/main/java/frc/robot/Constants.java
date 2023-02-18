// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.WarlordsLib.sendableRichness.SR_TrapezoidProfile;
import frc.util.COTSFalconSwerveConstants;
import frc.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kNominalVoltage = 12.0;
  public static final int kCANTimeoutMs = 250;
  public static final double kTimestepSeconds = 0.02;

  public static final double kRIOLoopTime = 0.02;

  // motor constants
  public static final double kFalconSensorUnitsPerRotation = 2048; // pulses per rotation
  public static final double kFalconWindingsResistanceOhms = 12.0 / 257;
  public static final double kFalconTorquePerAmp = 4.69 / 257;
  public static final double kFalconOutputUnitsFull = 1023;
  public static final double kFalconOutputUnitsPerVolt = kFalconOutputUnitsFull / kNominalVoltage;
  public static final double kFalconFreeSpeedRotationsPerSecond = 6380.0 / 60.0;
  public static final double kSecondsPer100Ms = 0.1;

  public static final double kNeoFreeSpeedRotationsPerSecond = 5676.0 / 60.0;
  public static final double kNeo550FreeSpeedRotationsPerSecond = 11000.0 / 60.0;

  public static final double k775FreeSpeedRotationsPerSecond = 18730.0 / 60.0;

  public static final class OIConstants {
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final double kDriverRightXDeadband = 0.1;
    public static final double kDriverLeftXDeadband = 0.1;
    public static final double kDriverLeftYDeadband = 0.1;

    public static final double kTriggerThreshold = 0.1;
  }

  public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 9;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.4445;
        public static final double wheelBase = 0.6477;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.32 / 12); 
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 4; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 3;
          public static final int angleMotorID = 4;
          public static final int canCoderID = 12;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-20);   
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, false);
      }

      /* Front Right Module - Module 1 */
      public static final class Mod1 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 1; 
          public static final int angleMotorID = 2;
          public static final int canCoderID = 11;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-134.485901);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, false);
      }
      
      /* Back Left Module - Module 2 */
      public static final class Mod2 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 5;
          public static final int angleMotorID = 6;
          public static final int canCoderID = 10;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-36.325378);
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, true);
      }

      /* Back Right Module - Module 3 */
      public static final class Mod3 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 7;
          public static final int angleMotorID = 8;
          public static final int canCoderID = 13;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-53.407288);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, true);
      }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPAutoXController = 4;
        public static final double kIAutoXController = 0.05;
        public static final double kDAutoXController = 0.2;
        public static final double kPAutoYController = 4;
        public static final double kIAutoYController = 0.05;
        public static final double kDAutoYController = 0.2;

        public static final double kPAutoThetaController = -7.5;
        public static final double kIAutoThetaController = 0.1;
        public static final double kDAutoThetaController = 0.1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final SR_TrapezoidProfile.Constraints kThetaControllerConstraints =
            new SR_TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
 public static final class ElevatorConstants {
        public static final int kElevatorPortLeft = 14;
        public static final int kElevatorPortRight =  15;

        public static final double kElevatorBottomStop = 0;
        public static final double kElevatorTopStop = 0.8763;

    
        public static final double kElevatorSupplyCurrentLimitAmps = 35;
        public static final double kElevatorSupplyCurrentThresholdAmps = 40;
        public static final double kElevatorSupplyCurrentThresholdTimeSecs = 0.1;
        public static final double kElevatorStatorCurrentLimitAmps = 50;
        public static final double kElevatorStatorCurrentThresholdAmps = 55;
        public static final double kElevatorStatorCurrentThresholdTimeSecs = 0.05;

        public static final double kElevatorGearRatio = 3.86;
        public static final double kSprocketCircumference = 0.032258 * Math.PI;

        public static final double kDistancePerMotorRev = kSprocketCircumference/kElevatorGearRatio;
        public static final double kDistancePerPulse = kDistancePerMotorRev/kFalconSensorUnitsPerRotation;

        public static final double kElevatorFreeSpeedMetersPerSecond = kFalconFreeSpeedRotationsPerSecond / kElevatorGearRatio * kSprocketCircumference;
        public static final double kElevatorMaxSpeedMetersPerSecond = kElevatorFreeSpeedMetersPerSecond * 0.9;
        public static final double kElevatorMaxAccelerationMetersPerSecondSquared = 1.5;

        //velocity loop constants
        public static final double kSElevatorVolts = 0.095783;
        public static final double kGElevatorVolts = 0.2;
        public static final double kVElevatorVoltsSecondsPerMeter = 2.5;
        public static final double kAElevatorVoltsSecondsSquaredPerMeter = 0.1854;
        
        //position loop constants
        public static final double kPElevatorVoltsPerMeter = 80;
        public static final double kIElevatorVoltsPerMeter = 0;
        public static final double kDElevatorVoltSecondsPerMeter = 0;
        public static final SR_TrapezoidProfile.Constraints kElevatorControllerConstraints = new SR_TrapezoidProfile.Constraints(
            kElevatorMaxSpeedMetersPerSecond,
            kElevatorMaxAccelerationMetersPerSecondSquared);
        public static final double kElevatorControlLoopTimeSeconds = 0.01;

 

        //public static final double kFeedForwardVoltage = 0;
    }
}

