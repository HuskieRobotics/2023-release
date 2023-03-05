package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class AutoBalanceNonStop extends CommandBase {

  private static final double KP = 0.04;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double MAX_ANGLE_DEG = 9.0;

  private PIDController frontBack;
  private PIDController leftRight;
  private Drivetrain drivetrain;
  // private double feedforward;
  private double maxVelocity;
  private boolean finishWhenBalanced;
  private boolean balanced;

  public AutoBalanceNonStop(Drivetrain drivetrain, boolean finishWhenBalanced) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // this.feedforward = 0;
    this.frontBack = new PIDController(KP, KI, KD);
    this.leftRight = new PIDController(KP, KI, KD);
    this.finishWhenBalanced = finishWhenBalanced;
    this.maxVelocity = 1;
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/AutoBalanceNonStop", true);

    drivetrain.disableFieldRelative();
    drivetrain.disableXstance();
  }

  @Override
  public void execute() {

    if (Math.max(drivetrain.getPitch(), drivetrain.getRoll()) < MAX_ANGLE_DEG
        && Math.min(drivetrain.getPitch(), drivetrain.getRoll()) > -MAX_ANGLE_DEG) {
      drivetrain.setXStance();
      balanced = true;
    } else {
      drivetrain.disableXstance();
      double pitch = drivetrain.getPitch();
      double roll = drivetrain.getRoll();
      // Rotation2d yaw = drivetrain.getRotation();
      // double feedforwardX = Math.sin(yaw.getRadians()) * feedforward;
      // double feedforwardY = Math.cos(yaw.getRadians()) * feedforward;

      // double frontBackOutput = -Math.min(Math.max(frontBack.calculate(roll, 0), -maxVelocity),
      // maxVelocity);
      // double leftRightOutput = Math.min(Math.max(leftRight.calculate(pitch, 0), -maxVelocity),
      // maxVelocity);
      double frontBackOutput = -frontBack.calculate(roll, 0);
      double leftRightOutput = leftRight.calculate(pitch, 0);
      if (Math.abs(frontBackOutput) < maxVelocity) frontBackOutput = maxVelocity;
      if (Math.abs(leftRightOutput) < maxVelocity) leftRightOutput = maxVelocity;

      drivetrain.drive(
          frontBackOutput /*+ feedforwardX*/, leftRightOutput /*+ feedforwardY*/, 0, true, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().recordOutput("ActiveCommands/AutoBalanceNonStop", false);
  }

  @Override
  public boolean isFinished() {
    return finishWhenBalanced && balanced;
  }
}
