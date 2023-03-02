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
 * <p>Finished When: the manipulator is opened
 *
 * <p>At End: stop the manipulator motor at the open position, hitting mechanical hardstops
 */
public class ReleaseGamePiece extends CommandBase {
  private final Manipulator manipulator;

  public ReleaseGamePiece(Manipulator subsystem) {
    this.manipulator = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/ReleaseGamePiece", true);

    manipulator.enableBrakeMode(false);
    manipulator.openPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return manipulator.isOpened();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulator.stop();
    Logger.getInstance().recordOutput("ActiveCommands/ReleaseGamePiece", false);
  }
}
