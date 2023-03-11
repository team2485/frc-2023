// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.WarlordsLib.sendableRichness.SR_TrapezoidProfile;
import frc.util.COTSFalconSwerveConstants;
import frc.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kNominalVoltage = 12.0;
  public static final int kCANTimeoutMs = 250;
  public static final double kTimestepSeconds = 0.02;

  public static final double kRIOLoopTime = 0.02;

  // motor constants
  public static final double kFalconSensorUnitsPerRotation = 2048; // pulses per rotation
  public static final double kNeoSensorUnitsPerRotation = 42;
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

    public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = 0.4445;
    public static final double wheelBase = 0.6477;
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
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

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
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
    public static final double maxSpeed = 3; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 4; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constan  ts */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-25);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-134.485901);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-36.325378);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-56.407288);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }
  }

  public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
                                            // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kSlowerSpeedMetersPerSecond = 2.5;
    public static final double kSlowerAccelerationMetersPerSecondSquared = 2;

    public static final double kMaxAngularSpeedRadiansPerSecond = 2*Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2*Math.PI;

    public static final double kPAutoXController = 3;
    public static final double kIAutoXController = 0.05;
    public static final double kDAutoXController = 0;
    public static final double kPAutoYController = 3;
    public static final double kIAutoYController = 0.25;
    public static final double kDAutoYController = 0;

    public static final double kPAutoThetaController = -5;
    public static final double kIAutoThetaController = -0;
    public static final double kDAutoThetaController = -1;

    /* Constraint for the motion profilied robot angle controller */
    public static final SR_TrapezoidProfile.Constraints kThetaControllerConstraints = new SR_TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ElevatorConstants {
    public static final int kElevatorPortLeft = 14;
    public static final int kElevatorPortRight = 15;

    public static final double kElevatorBottomStop = 0;
    public static final double kElevatorTopStop = 0.8763;

    public static final double kElevatorTolerance = 0.02;

    public static final double kElevatorOvershootAmountMeters = 0.04;

    public static final double kElevatorSupplyCurrentLimitAmps = 55;
    public static final double kElevatorSupplyCurrentThresholdAmps = 60;
    public static final double kElevatorSupplyCurrentThresholdTimeSecs = 0.05;
    public static final double kElevatorStatorCurrentLimitAmps = 65;
    public static final double kElevatorStatorCurrentThresholdAmps = 70;
    public static final double kElevatorStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kElevatorGearRatio = 3.86;
    public static final double kSprocketCircumference = 0.032258 * Math.PI;

    public static final double kDistancePerMotorRev = kSprocketCircumference / kElevatorGearRatio;
    public static final double kDistancePerPulse = kDistancePerMotorRev / kFalconSensorUnitsPerRotation;

    public static final double kElevatorFreeSpeedMetersPerSecond = kFalconFreeSpeedRotationsPerSecond
        / kElevatorGearRatio * kSprocketCircumference;
    public static final double kElevatorMaxSpeedMetersPerSecond = kElevatorFreeSpeedMetersPerSecond * 0.9;
    public static final double kElevatorMaxAccelerationMetersPerSecondSquared = 1.5;

    // velocity loop constants
    public static final double kSElevatorVolts = 0.095783;
    public static final double kGElevatorVolts = 0.8;
    public static final double kVElevatorVoltsSecondsPerMeter = 4;
    public static final double kAElevatorVoltsSecondsSquaredPerMeter = 0.1854;

    // position loop constants
    public static final double kPElevatorVoltsPerMeter = 80;
    public static final double kIElevatorVoltsPerMeter = 20;
    public static final double kDElevatorVoltSecondsPerMeter = 0;
    public static final SR_TrapezoidProfile.Constraints kElevatorControllerConstraints = new SR_TrapezoidProfile.Constraints(
        kElevatorMaxSpeedMetersPerSecond,
        kElevatorMaxAccelerationMetersPerSecondSquared);
    public static final double kElevatorControlLoopTimeSeconds = 0.02;

    // public static final double kFeedForwardVoltage = 0;
  }

  public static final class WristConstants {
    public static final int kWristTalonPort = 16;

    public static final double kWristGearRatio = 48; // motor turns : output/full hood turns
    public static final double kWristRadiansPerMotorRev = 2 * Math.PI / kWristGearRatio;
    public static final double kWristRadiansPerPulse = kWristRadiansPerMotorRev / kFalconSensorUnitsPerRotation;

    public static final double kWristBottomPositionRadians = 0; // from horizontal
    public static final double kWristTopPositionRadians = 4.4;

    public static final double kIndexerSupplyCurrentLimitAmps = 25;
    public static final double kIndexerSupplyCurrentThresholdAmps = 30;
    public static final double kIndexerSupplyCurrentThresholdTimeSecs = 0.1;

    public static final double kIndexerStatorCurrentLimitAmps = 40;
    public static final double kIndexerStatorCurrentThresholdAmps = 45;
    public static final double kIndexerStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kWristTolerance = 0.1;

    // Wrist characterization constants
    public static final double kSWristVolts = 0.083516;
    public static final double kGWristVolts = 0.3281;
    public static final double kVWristVoltSecondsPerRadian = 0.81827;
    public static final double kAWristVoltSecondsSquaredPerRadian = 0.029701;

    public static final double kWristMaxSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kWristMaxAccelerationRadiansPerSecondSquared = Math.PI;

    public static final SR_TrapezoidProfile.Constraints kWristMotionProfileConstraints = new SR_TrapezoidProfile.Constraints(
        kWristMaxSpeedRadiansPerSecond, kWristMaxAccelerationRadiansPerSecondSquared);
    // Wrist PID constants
    public static final double kPWrist = 3;
    public static final double kIWrist = 4;
    public static final double kDWrist = 0.25;
    public static final double kWristControllerPositionTolerance = 0;

  }

  public static final class GripperConstants {
    public static final int kGripperSparkPort = 17;

    public static final double kGripperOpenPosMeters = 0;
    public static final double kGripperClosedPosMeters = 3;

    public static final double kGripperControllerPositionTolerance = 0;
    public static final double kGripperOpenPositionSetpoint = 0;
    public static final int kGripperCurrentLimit = 40;
    public static final double kGripperStoppedVelocityTolerance = 0;
    public static final double kPieceDetectionTolerance = 0.1;
    public static final double kCubeEncoderDistance = 0.95;
    public static final double kConeEncoderThreshold = 1.1;

    public static final double kGripperGearRatio = 25;
    public static final double kGripperRadiansPerMotorRev = 2 * Math.PI / kGripperGearRatio;
    public static final double kGripperRadiansPerPulse = kGripperRadiansPerMotorRev / kNeoSensorUnitsPerRotation;

  }

  public static final class TelescopeConstants {

    public static final int kTelescopePort = 18;
    public static final int kTelescopeSmartCurrentLimitAmps = 55;
    public static final int kTelescopeImmediateCurrentLimitAmps = 65;

    public static final double kTelescopeTolerance = 0.03;

    public static final double kTelescopeMaxPosition = 1.1;

    public static final double kSTelescopeVolts = 0.01;
    public static final double kVTelescopeVoltSecondsPerMeter = 0.2;
    public static final double kATelescopeVoltSecondsSquaredPerMeter = 0.02;

    public static final double kMaxVelocity = 2.5;
    public static final double kMaxAcceleration = 1.5;

    public static final double kPTelescope = 45;
    public static final double kITelescope = 25;
    public static final double kDTelescope = 0;
    public static final SR_TrapezoidProfile.Constraints kMotionProfileConstraints = new SR_TrapezoidProfile.Constraints(
        kMaxVelocity, kMaxAcceleration);
    public static final double kTelescopeControlLoopTimeSeconds = 0.01;

    public static final double kTelescopeStartPostion = 0;

    public static final double kTelescopeGearRatio = 5;
    public static final double kPulleyCircumference = 0.0222 * Math.PI;
    public static final double kDistancePerMotorRev = kPulleyCircumference / kTelescopeGearRatio;

  }

  public static final class IntakeArmConstants {

    public static final int kIntakeArmPortLeft = 19; // tbd
    public static final int kIntakeArmPortRight = 20; // tbd

    public static final double kPIntakeArm = 2;
    public static final double kIIntakeArm = 0;
    public static final double kDIntakeArm = 0;
    public static final frc.WarlordsLib.sendableRichness.SR_TrapezoidProfile.Constraints kMotionProfileConstraints = new frc.WarlordsLib.sendableRichness.SR_TrapezoidProfile.Constraints(
        kFalconSensorUnitsPerRotation, kCANTimeoutMs);

    public static final double kGearRatio = 20;

    public static double kIntakeArmRetractedPositionRadians = 0;
    public static double kIntakeArmDeployedPositionRadians = 1.1; // radians betwween open and closoed pos.

    public static final double kIntakeArmSupplyCurrentLimitAmps = 35;
    public static final double kIntakeArmSupplyCurrentThresholdAmps = 40;
    public static final double kIntakeArmSupplyCurrentThresholdTimeSecs = 0.1;

    public static final double kIntakeArmStatorCurrentLimitAmps = 45;
    public static final double kIntakeArmStatorCurrentThresholdAmps = 50;
    public static final double kIntakeArmStatorCurrentThresholdTimeSecs = 0.05;

    public static double kMaxPositionMeters = kIntakeArmDeployedPositionRadians;

    // public static double kSprocketCrcumference = (kSprocketRadius*2)* Math.PI;
    // //TODO: ALL VARIABLES HERE ARE PLACEHOLDER

    public static final double kRadiansPerMotorRev = 2 * Math.PI / kGearRatio;
    public static final double kRadiansPerPulse = kRadiansPerMotorRev / kFalconSensorUnitsPerRotation;
  }

  public static final class IntakeConstants {

    public static final int kIntakeSparkPort = 21;
    public static final double kIntakeLoopTimeSeconds = 0.02;
    public static final int kIntakeSmartCurrentLimitAmps = 20;
    public static final int kIntakeImmediateCurrentLimitAmps = 25;

    public static final double kIntakeGearRatio = 2; // motor turns : output/full hood turns

    public static final double kIntakeFreeSpeedRotationsPerSecond = kNeoFreeSpeedRotationsPerSecond / kIntakeGearRatio;

    public static final double kIntakeTopWheelDiameterMeters = 0.1016; // 4 in
    public static final double kIntakeBottomWheelDiameterMeters = 0.1524; // 6 in
    public static final double kIntakeBottomWheelCircumferenceMeters = 0.1524 * Math.PI;
    public static final double kIntakeDefaultSpeedRotationsPerSecond = 9;

    public static final double kSIntakeVolts = 0.1;
    public static final double kVIntakeVoltSecondsPerMeter = 0.3;
    public static final double kAIntakeVoltSecondsSquaredPerMeter = 0.01;

  }

  public static final class MagazineConstants {
    public static final double kIntakeTopWheelDiameterMeters = 0.1016;
    public static final int kMagazineTalonPort = 22;
    public static final double kMagazineLoopTimeSeconds = 0.02;
    public static final int kMagazineSmartCurrentLimitAmps = 25;
    public static final int kMagazineImmediateCurrentLimitAmps = 30;

    public static final double kMagazineSupplyCurrentLimitAmps = 25;
    public static final double kMagazineSupplyCurrentThresholdAmps = 30;
    public static final double kMagazineSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kMagazineStatorCurrentLimitAmps = 40;
    public static final double kMagazineStatorCurrentThresholdAmps = 45;
    public static final double kMagazineStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kMagazineGearRatio = 4; // motor turns : output/full hood turns

    public static final double kMagazineFreeSpeedRotationsPerSecond = kNeoFreeSpeedRotationsPerSecond
        / kMagazineGearRatio;

    public static final double kMagazineEntryWheelDiameterMeters = 0.0508; // 2 inches

    public static final double kMagazineIntakeSpeedRatio = kIntakeTopWheelDiameterMeters
        / kMagazineEntryWheelDiameterMeters;

    public static final double kMagazineDefaultSpeedRotationsPerSecond = 4.5;

    public static final double kSMagazineVolts = 0.1;
    public static final double kVMagazineVoltSecondsPerMeter = 0.3;
    public static final double kAMagazineVoltSecondsSquaredPerMeter = 0.01;

    public static final double kMagazineVelocityToleranceRotationsPerSecond = 0.5;

  }

  public static final class VisionConstants {
    public static final String kCameraName = "photonvision";

    public static final TrapezoidProfile.Constraints kXConstraints = new TrapezoidProfile.Constraints(1, 2);
    public static final TrapezoidProfile.Constraints kYConstraints = new TrapezoidProfile.Constraints(.5, 2);
    public static final TrapezoidProfile.Constraints kOmegaConstraints = new TrapezoidProfile.Constraints(3, 8);

    // TODO: ensure validity of measurements
    //public static final Transform3d kCameraToRobot = new Transform3d(new Translation3d(.257175, .1635125 * .5, .47625),
    //    new Rotation3d());

    public static final Transform3d kCameraToRobot = new Transform3d(new Translation3d(.2225, .2065, .4635), new Rotation3d());

    // public static final int kTagOfInterest = 1;
    // public static final Transform2d kTagToGoal = new Transform2d(new
    // Translation2d(1, 0),
    // Rotation2d.fromDegrees(180.0));

    public static final double kOffsetToNextScoringStation = 0.39158333333164;
  }
}
