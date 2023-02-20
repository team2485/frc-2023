// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.motorcontrol.base.WPI_SparkMax;
import static frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private final WPI_SparkMax gripperMotor = new WPI_SparkMax(GripperConstants.kGripperMotorID, MotorType.kBrushless);
  private final RelativeEncoder gripperEncoder;
  private final PIDController gripperController = new PIDController(0, 0, 0);

  public enum PieceType {
    kCone,
    kCube,
    kNone,
  }

  private PieceType currentPieceType = PieceType.kNone;

  public Gripper() {
    gripperMotor.setSmartCurrentLimit(GripperConstants.kGripperCurrentLimit);
    gripperEncoder = gripperMotor.getEncoder();
    // some people were saying that this method was not properly implemented and does nothing
    // it probably works now as those complaints were from 2020 but check this if there are issues 
    gripperEncoder.setPositionConversionFactor(GripperConstants.kGripperGearRatio);
    gripperController.setTolerance(GripperConstants.kGripperPositionTolerance);
  }

  @Override
  public void periodic() {
    // only update the piece type when it would realistically change to prevent erratically swtiching between states as the encoder passes through them
    // This is from the current based approach
    //if (!isStalling()) return;

    if (!isStopped()) return;

    updateCurrentHeldPiece();
  }

  private void updateCurrentHeldPiece() {
    //if are fully closed or open enough we don't have anything
    if (Math.abs(getGripperPosition()) < GripperConstants.kPieceDetectionTolerance 
    || getGripperPosition() > GripperConstants.kGripperOpenPositionSetpoint - GripperConstants.kGripperPositionTolerance)
      setCurrentHeldPiece(PieceType.kNone);
    else if (Math.abs(getGripperPosition() - GripperConstants.kCubeEncoderDistance) < GripperConstants.kPieceDetectionTolerance)
      setCurrentHeldPiece(PieceType.kCube);
    else setCurrentHeldPiece(PieceType.kCone);
  }

  public void setGripperPower(double voltagePercentage) {
    gripperMotor.set(voltagePercentage);
  }

  public double getGripperPosition() {
    return gripperEncoder.getPosition();
  }

  public double getEncoderVelocity() {
    return gripperEncoder.getVelocity();
  }

  public void setPositionSetpoint(double setpoint) {
    gripperController.setSetpoint(setpoint);
  }

  public double getPIDControllerOutput() {
    return gripperController.calculate(getGripperPosition(), gripperController.getSetpoint());
  }

  public boolean isAtSetpoint() {
    return gripperController.atSetpoint();
  }

  public void setCurrentHeldPiece(PieceType pieceType) {
    currentPieceType = pieceType;
  }

  public PieceType getCurrenPieceType() {
    return currentPieceType;
  }

  public boolean isStopped()
  {
    return Math.abs(getEncoderVelocity()) < GripperConstants.kGripperStoppedVelocityTolerance;
  }

  // This is from the current based approach
  // public boolean isStalling() {
  //   // I don't know if the getOutputCurrent is given before or after the currentlimit is applied
  //   // Hopefully the >= works but if the gripper doesn't stop trying to close check here
  //   return gripperMotor.getOutputCurrent() >= GripperConstants.kGripperCurrentLimit;
  // }
}