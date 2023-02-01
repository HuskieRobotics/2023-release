package frc.robot.commands.AutoBalance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SimpleAutoBalanceLeftRight extends PIDCommand {
  private Drivetrain drivetrain;

  public SimpleAutoBalanceLeftRight(Drivetrain drivetrain) {
    super(
        new PIDController(1, 0, 0),
        drivetrain::getPitch,
        0,
        output -> drivetrain.drive(0, output, 0));
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  /*
   *     __X_
   *    |Frnt|
   *    -Y   Y
   *    |_-X_|
   *
   */

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    drivetrain.enableXstance();
    super.end(interrupted);
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * gyros pitch is between two value determined in constants.
   */
  @Override
  public boolean isFinished() {
    return drivetrain.getPitch() < 7 && drivetrain.getPitch() > -7;
  }
}
