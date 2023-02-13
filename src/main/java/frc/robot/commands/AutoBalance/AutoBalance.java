package frc.robot.commands.AutoBalance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalance extends CommandBase {

  private Drivetrain drivetrain;

  public AutoBalance(Drivetrain d) {
    drivetrain = d;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.disableFieldRelative();
    double yaw = Math.abs(drivetrain.getRotation().getDegrees() % 360.0);
    // double pitch = drivetrain.getPitch();
    // double roll = drivetrain.getRoll();
    // if (Math.max(pitch, roll) > 1) {
    //   if (Math.abs(pitch) > Math.abs(roll)) {
    //     new NewAutoBalanceFrontBack(drivetrain);
    //   } else {
    //     new NewAutoBalanceLeftRight(drivetrain);
    //   }
    // }

    if ((yaw >= -15.0 && yaw < 15.0) || (yaw >= 165.0 && yaw < 195.0)) {
      new NewAutoBalanceFrontBack(drivetrain);
    } else if ((yaw >= 75.0 && yaw < 105.0) || (yaw >= 255.0 && yaw < 285.0)) {
      new NewAutoBalanceLeftRight(drivetrain);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  private class NewAutoBalanceFrontBack extends PIDCommand {
    private Drivetrain drivetrain;

    public NewAutoBalanceFrontBack(Drivetrain drivetrain) {
      super(
          new PIDController(1, 0, 0),
          drivetrain::getRoll,
          0,
          output -> drivetrain.drive(output, 0, 0, true, false));
      this.drivetrain = drivetrain;
    }

    /**
     * This method will be invoked when this command finishes or is interrupted. It stops the motion
     * of the drivetrain.
     *
     * @param interrupted true if the command was interrupted by another command being scheduled
     */
    @Override
    public void end(boolean interrupted) {
      drivetrain.enableXstance();
      super.end(interrupted);
    }

    /**
     * This method is invoked at the end of each Command Scheduler iteration. It returns true when
     * the gyros pitch is between two value determined in constants.
     */
    @Override
    public boolean isFinished() {
      return drivetrain.getRoll() < .1 && drivetrain.getRoll() > -.1;
    }
  }

  private class NewAutoBalanceLeftRight extends PIDCommand {
    private Drivetrain drivetrain;

    public NewAutoBalanceLeftRight(Drivetrain drivetrain) {
      super(
          new PIDController(1, 0, 0),
          drivetrain::getRoll,
          0,
          output -> drivetrain.drive(0, output, 0, true, false));
      this.drivetrain = drivetrain;
    }

    /**
     * This method will be invoked when this command finishes or is interrupted. It stops the motion
     * of the drivetrain.
     *
     * @param interrupted true if the command was interrupted by another command being scheduled
     */
    @Override
    public void end(boolean interrupted) {
      drivetrain.enableXstance();
      super.end(interrupted);
    }

    /**
     * This method is invoked at the end of each Command Scheduler iteration. It returns true when
     * the gyros pitch is between two value determined in constants.
     */
    @Override
    public boolean isFinished() {
      return drivetrain.getPitch() < .1 && drivetrain.getPitch() > -.1;
    }
  }
}
