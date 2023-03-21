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

  private static final double KP = 0.1;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double MAX_ANGLE_DEG = 8.0;
  private static final double MAX_VELOCITY = 1.0;
  private static final double ANGLE_HYSTERESIS_DEGREES = 7.0;
  private static final double ADJUSTMENT_THRESHOLD_METERS = 0.2;

  private double initDriveVelocity;
  private PIDController frontBack;
  private PIDController leftRight;
  private Drivetrain drivetrain;
  private LEDs led;
  private boolean finishWhenBalanced;
  private boolean comingFromGrid;
  private Timer timer;
  private double timeout;
  private double initialX;

  private enum State {
    INIT,
    BALANCING,
    BALANCED,
    ADJUSTED,
    FINISHED
  };

  private State state = State.INIT;

  // FIXME: Adjust this value for the timeout we determine
  private static final TunableNumber tuneTimeout = new TunableNumber("AutoBalance/timeout", 40.0);
  private static final TunableNumber maxAngle =
      new TunableNumber("AutoBalance/threshold", MAX_ANGLE_DEG);
  private static final TunableNumber initDriveVelocityTunable =
      new TunableNumber("AutoBalance/initDriveVelocity", MAX_VELOCITY);
  private static final TunableNumber maxDriveVelocityTunable =
      new TunableNumber("AutoBalance/maxDriveVelocity", MAX_VELOCITY);

  private static final TunableNumber angleHysteresis =
      new TunableNumber("AutoBalance/angleHysteresis", ANGLE_HYSTERESIS_DEGREES);
  private static final TunableNumber adjustmentThreshold =
      new TunableNumber("AutoBalance/adjustmentThreshold", ADJUSTMENT_THRESHOLD_METERS);
  private static final TunableNumber AutoBalanceP = new TunableNumber("AutoBalance/p", KP);

  public AutoBalance(Drivetrain drivetrain, boolean finishWhenBalanced, LEDs led) {
    this(drivetrain, finishWhenBalanced, led, true);
  }

  public AutoBalance(
      Drivetrain drivetrain, boolean finishWhenBalanced, LEDs led, boolean comingFromGrid) {
    this.drivetrain = drivetrain;
    this.led = led;
    this.timer = new Timer();
    addRequirements(drivetrain);
    this.finishWhenBalanced = finishWhenBalanced;
    this.comingFromGrid = comingFromGrid;
  }

  @Override
  public void initialize() {
    this.frontBack = new PIDController(AutoBalanceP.get(), KI, KD);
    this.leftRight = new PIDController(AutoBalanceP.get(), KI, KD);
    Logger.getInstance().recordOutput("ActiveCommands/AutoBalanceNonStop", true);
    if (this.finishWhenBalanced) {
      this.timeout = tuneTimeout.get();
    } else {
      this.timeout = 1000000.0;
    }
    this.state = State.INIT;
    this.timer.restart();
    drivetrain.disableFieldRelative();
    drivetrain.disableXstance();
    led.changeAnimationTo(AnimationTypes.BLUE);

    if (comingFromGrid) {
      initDriveVelocity = initDriveVelocityTunable.get();
    } else {
      initDriveVelocity = -initDriveVelocityTunable.get();
    }
  }

  @Override
  public void execute() {
    Logger.getInstance().recordOutput("AutoBalanceNonStop/time", this.timer.get());
    Logger.getInstance().recordOutput("AutoBalanceNonStop/state", stateToString(this.state));

    // first check for state transitions
    if (state == State.INIT) {
      // check if we have started climbing the charging station
      if (Math.max(drivetrain.getPitch(), drivetrain.getRoll())
              > maxAngle.get() + angleHysteresis.get()
          || Math.min(drivetrain.getPitch(), drivetrain.getRoll())
              < -maxAngle.get() - angleHysteresis.get()) {
        state = State.BALANCING;
      }
    } else if (state == State.BALANCING) {
      // check if the charging station is leveling
      if (finishWhenBalanced
          && Math.max(drivetrain.getPitch(), drivetrain.getRoll()) < maxAngle.get()
          && Math.min(drivetrain.getPitch(), drivetrain.getRoll()) > -maxAngle.get()) {
        state = State.BALANCED;
        initialX = drivetrain.getPose().getX();
      }
    } else if (state == State.BALANCED) {
      // check if we have adjusted our position to stay balanced
      if (Math.abs(drivetrain.getPose().getX() - initialX) > adjustmentThreshold.get()) {
        state = State.FINISHED;
      }
    }

    // execute based on the current state
    if (state == State.INIT) {
      drivetrain.drive(initDriveVelocity, 0, 0, false, true);
    } else if (state == State.BALANCING) {
      double pitch = drivetrain.getPitch();
      double roll = drivetrain.getRoll();
      // Rotation2d yaw = drivetrain.getRotation();
      // double feedforwardX = Math.sin(yaw.getRadians()) * feedforward;
      // double feedforwardY = Math.cos(yaw.getRadians()) * feedforward;

      double frontBackOutput = -frontBack.calculate(roll, 0);
      double leftRightOutput = leftRight.calculate(pitch, 0);
      if (Math.abs(frontBackOutput) > maxDriveVelocityTunable.get())
        frontBackOutput = Math.copySign(maxDriveVelocityTunable.get(), frontBackOutput);
      if (Math.abs(leftRightOutput) > maxDriveVelocityTunable.get())
        leftRightOutput = Math.copySign(maxDriveVelocityTunable.get(), leftRightOutput);

      drivetrain.drive(
          frontBackOutput /*+ feedforwardX*/, leftRightOutput /*+ feedforwardY*/, 0, true, false);
    } else if (state == State.BALANCED) {
      drivetrain.drive(-initDriveVelocity / 2.0, 0, 0, false, true);
    } else if (state == State.FINISHED) {
      drivetrain.stop();
      drivetrain.enableXstance();
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
    return (state == State.FINISHED) || (this.timer.hasElapsed(this.timeout));
  }

  private String stateToString(State state) {
    if (state == State.INIT) {
      return "init";
    } else if (state == State.BALANCED) {
      return "balanced";
    } else if (state == State.BALANCING) {
      return "balancing";
    } else if (state == State.ADJUSTED) {
      return "adjusted";
    } else if (state == State.FINISHED) {
      return "finished";
    } else {
      return "";
    }
  }
}
