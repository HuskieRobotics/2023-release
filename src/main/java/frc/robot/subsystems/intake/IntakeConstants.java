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

  public static final boolean DEBUGGING = true;

  public static final boolean TESTING = true;
  public static final boolean TUNING = true;
  public static final String SUBSYSTEM_NAME = "Intake";

  public static final int INTAKE_ROTATION_MOTOR_CAN_ID = 2;
  public static final int INTAKE_ROLLER_MOTOR_CAN_ID = 3;
  public static final double INTAKE_ROTATION_GEAR_RATIO = 75.0 / 2.0;

  public static final double ROTATION_POSITION_PID_F = 0;
  public static final double ROTATION_POSITION_PID_P = 0;
  public static final double ROTATION_POSITION_PID_I = 0;
  public static final double ROTATION_POSITION_PID_D = 0;
  public static final double ROTATION_POSITION_PID_I_ZONE = 0;
  public static final double ROTATION_POSITION_PID_PEAK_OUTPUT = 0;

  public static final double ROTATION_ACCELERATION = 0;
  public static final double MAX_ROTATION_VELOCITY = 0;
  public static final int SCURVE_STRENGTH = 0;
  public static final int TIMEOUT_MS = 0;
  public static final double ROTATION_FEEDFORWARD = 0;
  public static final double INTAKE_DEFAULT_ROLLER_POWER = 0;

  public static final double INTAKE_ROTATION_CUBE_POSITION = 0;
  public static final double INTAKE_ROTATION_CONE_POSITION = 0;
  public static final double INTAKE_ROTATION_NEUTRAL_POSITION = 0;
  public static final double INTAKE_ROTATION_TOLERANCE = 0;

  public static final double CUBE_INTAKE_ROLLER_POWER = 0;
  public static final double CUBE_INTAKE_ROTATION = 0;

  public static final double CHARGE_STATION_ROLLER_POWER = 0;
  public static final double CHARGE_STATION_ROTATION = 0;

  public static final double RETRACTED_ROLLER_POWER = 0;
  public static final double RETRACTED_ROTATION = 0;

  public static final double SCORING_ROLLER_POWER = 0;
  public static final double SCORING_ROTATION = 0;

  public static final double PUSH_CONE_CUBE_ROLLER_POWER = 0;
  public static final double PUSH_CONE_CUBE_ROTATION = 0;

  public static final double INTAKE_ROTATION_RETRACTION_POWER = 0;
  public static final double INTAKE_ROTATION_MANUAL_CONTROL_POWER = 0.2;

  public static final double INTAKE_ROTATION_DEFAULT_POSITION = 0;

  public static final int SLOT_INDEX = 0;

  public static final boolean INTAKE_ROLLER_MOTOR_INVERTED = false;
  public static final boolean INTAKE_ROTATION_MOTOR_INVERTED = false;

  // FIXME - change to 20 after testing if limit works
  public static final int INTAKE_ROLLER_CURRENT_LIMIT = 5;
  public static final double INTAKE_ROTATION_CURRENT_THRESHOLD = 29.5;

  public static final StatorCurrentLimitConfiguration INTAKE_ROTATION_CURRENT_LIMIT =
      new StatorCurrentLimitConfiguration(true, 30, 50, 0.5);
}