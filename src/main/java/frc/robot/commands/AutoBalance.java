package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.AnimationTypes;
import frc.robot.subsystems.leds.LEDs.RobotStateColors;
import org.littletonrobotics.junction.Logger;

public class AutoBalance extends CommandBase {

  private static final double KP = 0.04;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double MAX_ANGLE_DEG = 10.0;
  private static final double MAX_VELOCITY = 0.5;

  private PIDController frontBack;
  private PIDController leftRight;
  private Drivetrain drivetrain;
  private LEDs led;
  // private double feedforward;
  private boolean finishWhenBalanced;
  private boolean balanced;
  private boolean started;
  private Timer timer;
  private double timeout;
  // FIXME: Adjust this value for the timeout we determine
  private static final TunableNumber tuneTimeout = new TunableNumber("AutoBalance/timeout", 40.0);
  private static final TunableNumber maxAngle = new TunableNumber("AutoBalance/threshold", 10);

  public AutoBalance(Drivetrain drivetrain, boolean finishWhenBalanced, LEDs led) {
    this.drivetrain = drivetrain;
    this.led = led;
    this.timer = new Timer();
    addRequirements(drivetrain);
    // this.feedforward = 0;
    this.frontBack = new PIDController(KP, KI, KD);
    this.leftRight = new PIDController(KP, KI, KD);
    this.finishWhenBalanced = finishWhenBalanced;
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/AutoBalanceNonStop", true);
    if (this.finishWhenBalanced) {
      this.timeout = tuneTimeout.get();
    } else {
      this.timeout = 1000000.0;
    }
    this.balanced = false;
    this.timer.restart();
    drivetrain.disableFieldRelative();
    drivetrain.disableXstance();
    led.changeAnimationTo(AnimationTypes.BLUE);
    if (Math.max(drivetrain.getPitch(), drivetrain.getRoll()) < maxAngle.get()
        && Math.min(drivetrain.getPitch(), drivetrain.getRoll()) > -maxAngle.get()) {
      started = false;
    } else {
      started = true;
    }
  }

  @Override
  public void execute() {
    Logger.getInstance().recordOutput("AutoBalanceNonStop/time", this.timer.get());
    if (!started) {
      // FIXME: Will only work if robot is driving forward onto the charge station. maybe use
      // overrideFieldRelative set to true to always go right direction
      drivetrain.drive(MAX_VELOCITY, 0, 0, false, false);
      if (!(Math.max(drivetrain.getPitch(), drivetrain.getRoll()) < maxAngle.get()
          && Math.min(drivetrain.getPitch(), drivetrain.getRoll()) > -maxAngle.get())) {
        started = true;
      }
    }
    if (Math.max(drivetrain.getPitch(), drivetrain.getRoll()) < maxAngle.get()
        && Math.min(drivetrain.getPitch(), drivetrain.getRoll()) > -maxAngle.get()) {
      drivetrain.setXStance();
      balanced = true;
    } else {
      drivetrain.disableXstance();
      double pitch = drivetrain.getPitch();
      double roll = drivetrain.getRoll();
      // Rotation2d yaw = drivetrain.getRotation();
      // double feedforwardX = Math.sin(yaw.getRadians()) * feedforward;
      // double feedforwardY = Math.cos(yaw.getRadians()) * feedforward;

      double frontBackOutput = -frontBack.calculate(roll, 0);
      double leftRightOutput = leftRight.calculate(pitch, 0);
      if (Math.abs(frontBackOutput) > MAX_VELOCITY)
        frontBackOutput = Math.copySign(MAX_VELOCITY, frontBackOutput);
      if (Math.abs(leftRightOutput) > MAX_VELOCITY)
        leftRightOutput = Math.copySign(MAX_VELOCITY, leftRightOutput);

      drivetrain.drive(
          frontBackOutput /*+ feedforwardX*/, leftRightOutput /*+ feedforwardY*/, 0, true, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.disableXstance();
    drivetrain.enableFieldRelative();
    Logger.getInstance().recordOutput("ActiveCommands/AutoBalanceNonStop", false);
    led.changeTopStateColor(RobotStateColors.WHITE);
  }

  @Override
  public boolean isFinished() {
    return (started && finishWhenBalanced && balanced) || (this.timer.hasElapsed(this.timeout));
  }
}
