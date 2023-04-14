package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class IntakeConstants {

  public enum Position {
    RETRACTED,
    CHARGE_STATION,
    CUBE_INTAKE,
    PUSH_CONE_CUBE,
    SCORING
  }

  // FIXME: tune all of these
  public static final boolean DEBUGGING = false;

  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Intake";

  public static final int INTAKE_ROTATION_MOTOR_CAN_ID = 2;
  public static final int INTAKE_ROLLER_MOTOR_CAN_ID = 3;
  // FIXME: get correct gear ratio
  public static final double INTAKE_ROTATION_GEAR_RATIO = 16.0;

  public static final double INTAKE_DEFAULT_ROLLER_POWER = 0.3;
  public static final double INTAKE_ROTATION_MANUAL_CONTROL_POWER = 0.2;

  public static final double ROTATION_POSITION_PID_F = 0.004;
  public static final double ROTATION_POSITION_PID_P = 0.08;
  public static final double ROTATION_POSITION_PID_I = 0;
  public static final double ROTATION_POSITION_PID_D = 0;
  public static final double ROTATION_POSITION_PID_I_ZONE = 0;
  public static final double ROTATION_POSITION_PID_PEAK_OUTPUT = 0.5;
  public static final double ROTATION_FEEDFORWARD = 0;

  public static final double ROTATION_CURRENT_LIMIT = 40;
  public static final double ROLLER_CURRENT_LIMIT = 50;

  public static final double ROLLER_THRESHOLD_ITERATIONS = 10;
  public static final double DEPLOY_THRESHOLD_ITERATIONS = 10;

  public static final int SLOT_INDEX = 0;

  public static final boolean INTAKE_ROLLER_MOTOR_INVERTED = false;
  public static final boolean INTAKE_ROTATION_MOTOR_INVERTED = true;

  public static final StatorCurrentLimitConfiguration INTAKE_ROTATION_CURRENT_LIMIT =
      new StatorCurrentLimitConfiguration(true, ROTATION_CURRENT_LIMIT, 50, 0.5);
  public static final StatorCurrentLimitConfiguration INTAKE_ROLLER_CURRENT_LIMIT =
      new StatorCurrentLimitConfiguration(true, ROLLER_CURRENT_LIMIT, 50, 0.5);

  public static final double ROTATION_THRESHOLD_CURRENT = ROTATION_CURRENT_LIMIT - 0.5;
  public static final double ROLLER_THRESHOLD_CURRENT = ROLLER_CURRENT_LIMIT - 0.5;
  public static final int OPEN_THRESHOLD_ITERATIONS = 5;

  public static final double DEPLOY_POWER = 0.5;

  public static final double RETRACT_POWER = -0.5;

  public static final double INTAKE_ROTATION_TOLERANCE = 1;
}
