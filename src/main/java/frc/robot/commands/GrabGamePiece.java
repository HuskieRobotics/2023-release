// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.Manipulator;
import org.littletonrobotics.junction.Logger;

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
    Logger.getInstance().recordOutput("ActiveCommands/GrabGamePiece", true);

    manipulator.enableBrakeMode(true);
    manipulator.openPosition();
    this.isOpened = false;
  }

  @Override
  public void execute() {
    if (!this.isOpened && manipulator.isOpened()) {
      this.isOpened = true;
      manipulator.stop();
    }

    if (this.isOpened && manipulator.isBlocked()) {
      manipulator.close();
    }
  }

  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().recordOutput("ActiveCommands/GrabGamePiece", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return manipulator.isClosed();
  }
}
