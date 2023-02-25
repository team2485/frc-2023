// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import io.github.oblarg.oblog.Loggable;

public class GamePieceStateMachine implements Loggable {

  public enum pieceState {
    kCone,
    kCube;
  }

  public enum heightState {
    kHigh,
    kMedium,
    kLow;
  }

  private pieceState m_pieceState;
  private heightState m_heightState;

  /** Creates a new GamePieceStateMachine. */
  public GamePieceStateMachine() {

    m_pieceState = pieceState.kCone;
    m_heightState = heightState.kLow;
  }

  public void setPieceState(pieceState state) {
    m_pieceState = state;
  }

  public Command setPieceStateCommand(pieceState state) {
    return new InstantCommand(() -> this.setPieceState(state));
  }

  public pieceState getPieceState() {
    return m_pieceState;
  }

  public void setHeightState(heightState state) {
    m_heightState = state;
  }

  public Command setHeightStateCommand(heightState state) {
    return new InstantCommand(() -> this.setHeightState(state));
  }

  public heightState getHeightState() {
    return m_heightState;
  }

}
