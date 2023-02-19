package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalanceNonStop extends CommandBase {

  private static final double KP = 0.04;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double MAX_ANGLE_DEG = 9.0;

  private PIDController frontBack;
  private PIDController leftRight;
  private Drivetrain drivetrain;
  private double feedforward;

  public AutoBalanceNonStop(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.feedforward = 0;
    frontBack = new PIDController(KP, KI, KD);
    leftRight = new PIDController(KP, KI, KD);
  }

  @Override
  public void initialize() {
    drivetrain.disableFieldRelative();
    drivetrain.disableXstance();
  }

  @Override
  public void execute() {
    if (Math.max(drivetrain.getPitch(), drivetrain.getRoll()) < MAX_ANGLE_DEG
        && Math.min(drivetrain.getPitch(), drivetrain.getRoll()) > -MAX_ANGLE_DEG) {
      drivetrain.setXStance();
    } else {
      drivetrain.disableXstance();
      double pitch = drivetrain.getPitch();
      double roll = drivetrain.getRoll();
      Rotation2d yaw = drivetrain.getRotation();
      double feedforwardX = Math.sin(yaw.getRadians()) * feedforward;
      double feedforwardY = Math.cos(yaw.getRadians()) * feedforward;
      double frontBackOutput = frontBack.calculate(roll, 0);
      double leftRightOutput = leftRight.calculate(pitch, 0);
      drivetrain.drive(frontBackOutput + feedforwardX, leftRightOutput + feedforwardY, 0, true);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
