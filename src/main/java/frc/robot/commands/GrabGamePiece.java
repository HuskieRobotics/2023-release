// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.Manipulator;

/**
 * This command, when executed,
 *
 * <p>Requires: the Manipulator subsystem (handled by superclass)
 *
 * <p>Finished When: the manipulator is closed
 *
 * <p>At End: leave the manipulator motor stalled to secure the game piece
 */
public class GrabGamePiece extends CommandBase {
  private final Manipulator manipulator;
  private boolean isOpened = false;

  public GrabGamePiece(Manipulator subsystem) {
    this.manipulator = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulator.enableBrakeMode(true);
    manipulator.openPosition();
    this.isOpened = false;
  }

  @Override
  public void execute() {
    if (!this.isOpened) {
      if (manipulator.isOpened()) {
        this.isOpened = true;
        manipulator.stop();
      }
    }

    if (this.isOpened && manipulator.isBlocked()) {
      manipulator.close();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return manipulator.isClosed();
  }
}
