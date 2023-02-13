package frc.robot.commands.AutoBalance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalanceOneCommand extends CommandBase {

  private Drivetrain drivetrain;
  private int direction;
  private PIDController controller;

  private static final int FORWARD_BACK = 0;
  private static final int LEFT_RIGHT = 1;
  private static final int BALANCED = -1;

  public AutoBalanceOneCommand(Drivetrain d) {
    drivetrain = d;
    direction = BALANCED;
    controller = new PIDController(.1, 0, 0);
    controller.setSetpoint(0);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.disableFieldRelative();
    double yaw = drivetrain.getRotation().getDegrees();
    // if (Math.max(pitch, roll) > 1) {
    //   if (Math.abs(drivetrain.getPitch()) > Math.abs(drivetrain.getRoll())) {
    //     direction = FORWARD_BACK;
    //   } else {
    //     direction = LEFT_RIGHT;
    //   }
    // }
    if ((yaw >= -15.0 && yaw < 15.0) || (yaw >= 165.0 && yaw < 195.0)) {
      direction = FORWARD_BACK;
    } else {
      direction = LEFT_RIGHT;
    }
  }

  @Override
  public void execute() {
    double speed;
    switch (direction) {
      case FORWARD_BACK:
        speed = controller.calculate(drivetrain.getPitch(), 0);
        drivetrain.drive(speed, 0, 0, true, false);

      case LEFT_RIGHT:
        speed = controller.calculate(drivetrain.getPitch(), 0);
        drivetrain.drive(speed, 0, 0, true, false);
    }
  }

  @Override
  public boolean isFinished() {
    double pitch = drivetrain.getPitch();
    double roll = drivetrain.getRoll();
    switch (direction) {
      case FORWARD_BACK:
        if (pitch < .1 && pitch > -0.1) {
          return true;
        }
      case LEFT_RIGHT:
        if (roll < .1 && roll > -0.1) {
          return true;
        }
      case BALANCED:
        return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.enableXstance();
    super.end(interrupted);
  }
}
