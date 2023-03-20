package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class ManipulatorConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ManipulatorConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final int MANIPULATOR_MOTOR_ID = 1;
  public static final int MANIPULATOR_SENSOR_ID = 0;
  public static final StatorCurrentLimitConfiguration MANIPULATOR_CURRENT_LIMIT_CLOSE =
      new StatorCurrentLimitConfiguration(true, 20, 40, 0.5);
  public static final StatorCurrentLimitConfiguration MANIPULATOR_CURRENT_LIMIT_OPEN =
      new StatorCurrentLimitConfiguration(true, 10, 20, 0.5);
  public static final double MANIPULATOR_POWER = 1;
  public static final double MANIPULATOR_OPEN_POSITION = 0;

  public static final boolean MANIPULATOR_MOTOR_INVERTED = false;
  public static final NeutralMode MANIPULATOR_MOTOR_NEUTRAL_MODE = NeutralMode.Brake;

  public static final double MANIPULATOR_KP = 0.1;
  public static final double MANIPULATOR_KI = 0.0;
  public static final double MANIPULATOR_KD = 0.0;

  public static final double CLOSE_THRESHOLD_CURRENT = 29.0;
  public static final double OPEN_THRESHOLD_CURRENT = 9.0;
  public static final int OPEN_THRESHOLD_ITERATIONS = 10;
}
