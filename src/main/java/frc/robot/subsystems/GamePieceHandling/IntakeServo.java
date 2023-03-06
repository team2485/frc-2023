// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GamePieceHandling;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeServo extends SubsystemBase implements Loggable {
  /** Creates a new Servo. */

  public static Servo m_servoRight = new Servo(8);
  public static Servo m_servoLeft = new Servo(9);

  public IntakeServo() {
    m_servoLeft.set(0.35);
    m_servoRight.set(0);
  }

  public static void release() {
    m_servoLeft.set(0.35);
    m_servoRight.set(0);
  }

  public static void lock() {
    m_servoLeft.set(0);
    m_servoRight.set(0.35);
  }

  public double getPosition() {
    return m_servoRight.get();
  }

  @Log(name = "servo locked?", tabName = "RobotContainer")
  public boolean isLocked() {
    return m_servoRight.get() == 0.35;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
