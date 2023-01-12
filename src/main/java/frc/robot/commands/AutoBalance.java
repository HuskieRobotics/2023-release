package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalance extends PIDCommand {
  private static int PIDGEON_ID = 0;
  private Drivetrain drivetrain;

  public AutoBalance(Drivetrain drivetrain) {
    super(
        new PIDController(0, 0, 0),
        drivetrain::getPidgeonAngle,
        0,
        output -> drivetrain.drive(output, 0, 0));
    this.drivetrain = drivetrain;
  }
}
