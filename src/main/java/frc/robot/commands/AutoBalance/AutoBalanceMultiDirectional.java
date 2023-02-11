package frc.robot.commands.AutoBalance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalanceMultiDirectional extends CommandBase {
  private PIDController frontBack;
  private PIDController leftRight;
  private Drivetrain drivetrain;
  private double feedforward;

  public AutoBalanceMultiDirectional(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.feedforward = 0;
    frontBack = new PIDController(.05, 0, 0);
    leftRight = new PIDController(.05, 0, 0);
  }

  @Override
  public void initialize() {
    drivetrain.disableFieldRelative();
    drivetrain.disableXstance();
  }

  @Override
  public void execute() {
    double pitch = drivetrain.getPitch();
    double roll = drivetrain.getRoll();
    Rotation2d yaw = drivetrain.getRotation();
    double feedforwardX = Math.sin(yaw.getRadians()) * feedforward;
    double feedforwardY = Math.cos(yaw.getRadians()) * feedforward;
    double frontBackOutput = frontBack.calculate(roll, 0);
    double leftRightOutput = leftRight.calculate(pitch, 0);
    drivetrain.drive(frontBackOutput + feedforwardX, leftRightOutput + feedforwardY, 0, true);
  }

  @Override
  public boolean isFinished() {
    return Math.max(drivetrain.getPitch(), drivetrain.getRoll()) < 10
        && Math.min(drivetrain.getPitch(), drivetrain.getRoll())
            > -10; // || (Math.abs(drivetrain.getPitch()) >=13.5 || Math.abs(drivetrain.getRoll())
    // >= 13.5);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    drivetrain.enableXstance();
  }
}
