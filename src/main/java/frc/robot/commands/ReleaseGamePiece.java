// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import java.util.function.BooleanSupplier;
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
  private final Timer releaseTimer = new Timer();
  private BooleanSupplier isConeSupplier;
  private final TunableNumber coneReleaseTime =
      new TunableNumber("ReleaseGamePiece/ConeReleaseTime", ManipulatorConstants.CONE_OPEN_TIME);
  private final TunableNumber cubeReleaseTime =
      new TunableNumber("ReleaseGamePiece/CubeReleaseTime", ManipulatorConstants.CUBE_OPEN_TIME);

  public ReleaseGamePiece(Manipulator subsystem, BooleanSupplier isConeSupplier) {
    this.manipulator = subsystem;
    this.isConeSupplier = isConeSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/ReleaseGamePiece", true);
    Logger.getInstance()
        .recordOutput("ReleaseGamePiece/isCone", this.isConeSupplier.getAsBoolean());
    releaseTimer.reset();
    releaseTimer.start();

    manipulator.open();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.isConeSupplier.getAsBoolean()) {
      return releaseTimer.get() > coneReleaseTime.get();
    } else {
      return releaseTimer.get() > cubeReleaseTime.get();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().recordOutput("ActiveCommands/ReleaseGamePiece", false);
  }
}
