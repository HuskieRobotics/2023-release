package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.*;
// FIXME ALL CONSTANTS

public class ElevatorConstants {

  /** Make constants for each positions */
  public static final boolean DEBUGGING = false;

  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Elevator";

  public enum Position {
    INVALID,

    CONE_STORAGE,
    CUBE_STORAGE,

    AUTO_STORAGE,

    CONE_INTAKE_FLOOR,
    CUBE_INTAKE_BUMPER,

    CONE_INTAKE_SHELF,
    CUBE_INTAKE_SHELF,
    CONE_INTAKE_CHUTE,
    CUBE_INTAKE_CHUTE,

    CONE_HYBRID_LEVEL,
    CONE_MID_LEVEL,
    CONE_HIGH_LEVEL,
    CUBE_HYBRID_LEVEL,
    CUBE_MID_LEVEL,
    CUBE_HIGH_LEVEL,
  }

  public static final double ROTATION_OFFSET = -Units.degreesToRadians(45.0);

  // FIXME: make tunable and tune
  public static final double ROTATION_EXTENSION_TIME_OFFSET_OUT = 0.25;
  public static final boolean APPLY_TIME_OFFSET_AT_START_OUT = false;
  public static final double ROTATION_EXTENSION_TIME_OFFSET_IN = 0.5;
  public static final boolean APPLY_TIME_OFFSET_AT_START_IN = true;

  public static final double CONE_HIGH_ROTATION_POSITION = 0.7271 + ROTATION_OFFSET;
  public static final double CONE_HIGH_EXTENSION_POSITION = 1.6901;

  public static final double CONE_MID_ROTATION_POSITION = 0.7777 + ROTATION_OFFSET;
  public static final double CONE_MID_EXTENSION_POSITION = 1.2103;

  public static final double CONE_HYBRID_ROTATION_POSITION = 0.9832 + ROTATION_OFFSET;
  public static final double CONE_HYBRID_EXTENSION_POSITION = 0.6823;

  public static final double CUBE_HIGH_ROTATION_POSITION =
      Units.degreesToRadians(90.0 - 52.0) + ROTATION_OFFSET;
  public static final double CUBE_HIGH_EXTENSION_POSITION = Units.inchesToMeters(58);

  public static final double CUBE_MID_ROTATION_POSITION =
      Units.degreesToRadians(90.0 - 51.55) + ROTATION_OFFSET;
  public static final double CUBE_MID_EXTENSION_POSITION = Units.inchesToMeters(40.63);

  public static final double CUBE_HYBRID_ROTATION_POSITION = CONE_HYBRID_ROTATION_POSITION;
  public static final double CUBE_HYBRID_EXTENSION_POSITION = CONE_HYBRID_EXTENSION_POSITION;

  public static final double CONE_STORAGE_ROTATION_POSITION = 1.2 + ROTATION_OFFSET;
  public static final double CONE_STORAGE_EXTENSION_POSITION = 0;

  public static final double CUBE_STORAGE_ROTATION_POSITION = CONE_STORAGE_ROTATION_POSITION;
  public static final double CUBE_STORAGE_EXTENSION_POSITION = CONE_STORAGE_EXTENSION_POSITION;

  public static final double STORAGE_EXTENSION_POSITION_TOLERANCE = 0.02;
  public static final double STORAGE_ROTATION_POSITION_TOLERANCE = 0.05;

  public static final double CONE_GROUND_INTAKE_ROTATION_POSITION =
      Units.degreesToRadians(8.02) + ROTATION_OFFSET;
  public static final double CONE_GROUND_INTAKE_EXTENSION_POSITION = Units.inchesToMeters(32);

  public static final double SHELF_ROTATION_POSITION = Units.degreesToRadians(60) + ROTATION_OFFSET;
  public static final double SHELF_EXTENSION_POSITION = Units.inchesToMeters(35);

  public static final double CONE_CHUTE_ROTATION_POSITION =
      Units.degreesToRadians(90.0 - 27.0) + ROTATION_OFFSET;
  public static final double CONE_CHUTE_EXTENSION_POSITION = Units.inchesToMeters(24);

  public static final double CUBE_CHUTE_ROTATION_POSITION =
      Units.degreesToRadians(90.0 - 27.0) + ROTATION_OFFSET;
  public static final double CUBE_CHUTE_EXTENSION_POSITION = Units.inchesToMeters(24);

  public static final double AUTO_STORAGE_ROTATION = CONE_STORAGE_ROTATION_POSITION;
  public static final double AUTO_STORAGE_EXTENSION = CONE_GROUND_INTAKE_EXTENSION_POSITION;

  public static final double HIGH_ROTATION_POSITION_TIME = 0;
  public static final double MID_ROTATION_POSITION_TIME = 0;
  public static final double HYBRID_ROTATION_POSITION_TIME = 0;
  public static final double CONE_INTAKE_ROTATION_POSITION_TIME = 0;
  public static final double CUBE_INTAKE_ROTATION_POSITION_TIME = 0;
  public static final double SHELF_ROTATION_POSITION_TIME = 0;
  public static final double CHUTE_ROTATION_POSITION_TIME = 0;

  public static final double HIGH_EXTENSION_POSITION_TIME = 0;
  public static final double MID_EXTENSION_POSITION_TIME = 0;
  public static final double HYBRID_EXTENSION_POSITION_TIME = 0;
  public static final double CONE_INTAKE_EXTENSION_POSITION_TIME = 0;
  public static final double CUBE_INTAKE_EXTENSION_POSITION_TIME = 0;
  public static final double SHELF_EXTENSION_POSITION_TIME = 0;
  public static final double CHUTE_EXTENSION_POSITION_TIME = 0;

  public static final int START_EXTENSION_POSITION_INCHES = 0;

  public static final double MIN_ROTATION_POSITION = 0.0;
  public static final double MAX_ROTATION_POSITION = 1.297;

  public static final int EXTENSION_ELEVATOR_MOTOR_CAN_ID = 5;
  public static final int ROTATION_ELEVATOR_MOTOR_CAN_ID = 19;

  public static final boolean EXTENSION_INVERTED = false;
  public static final double EXTENSION_POSITION_PID_F = 0.05;
  public static final double EXTENSION_POSITION_PID_P = 0.05;
  public static final double EXTENSION_POSITION_PID_I = 0.0;
  public static final double EXTENSION_POSITION_PID_D = 0.0;
  public static final double EXTENSION_POSITION_PID_I_ZONE = 0.0;
  public static final double EXTENSION_POSITION_PID_PEAK_OUTPUT = 1.0;
  public static final int EXTENSION_MAX_STALL_DURATION_CYCLES = 10;
  public static final double EXTENSION_MAX_STALL_VELOCITY_METERS_PER_SECOND = 0.01;

  public static final boolean ROTATION_INVERTED = false;
  public static final double ROTATION_POSITION_PID_F = 0.05;
  public static final double ROTATION_POSITION_PID_P = 7.5;
  public static final double ROTATION_POSITION_PID_I = 0.0;
  public static final double ROTATION_POSITION_PID_D = 0.0;
  public static final double ROTATION_POSITION_PID_I_ZONE = 0.0;
  public static final double ROTATION_POSITION_PID_PEAK_OUTPUT = 1.0;

  public static final int PIGEON_ID = 4;
  public static final double EXTENSION_SLOW_PEAK_OUTPUT = 0.15;

  public static final double MAX_EXTENSION_VELOCITY_METERS_PER_SECOND = 1.5;
  public static final double EXTENSION_ACCELERATION_METERS_PER_SECOND_PER_SECOND = 3.0;
  public static final double MAX_RETRACTION_VELOCITY_METERS_PER_SECOND = 1.5;
  public static final double RETRACTION_ACCELERATION_METERS_PER_SECOND_PER_SECOND = 3.0;

  public static final double FAST_ROTATION_VELOCITY_DEGREES_PER_SECOND = 50.0;
  public static final double MEDIUM_ROTATION_VELOCITY_DEGREES_PER_SECOND = 45.0;
  public static final double SLOW_ROTATION_VELOCITY_DEGREES_PER_SECOND = 25.0;
  public static final double FAST_ROTATION_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND = 100.0;
  public static final double MEDIUM_ROTATION_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND = 75.0;
  public static final double SLOW_ROTATION_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND = 50.0;

  public static final double ELEVATOR_EXTENSION_POSITION_TOLERANCE = .02;
  public static final double MAX_MANUAL_POWER_EXTENSION = 0.2;

  public static final double ELEVATOR_ROTATION_POSITION_TOLERANCE = .01;
  public static final double MAX_MANUAL_POWER_ROTATION = 0.3;

  public static final double EXTENSION_PULLEY_CIRCUMFERENCE = Units.inchesToMeters(1.128) * Math.PI;
  public static final double EXTENSION_GEAR_RATIO = 3.0;

  public static final int PIGEON_UNITS_PER_ROTATION = 8192;

  public static final int TIMEOUT_MS = 30;
  public static final int SLOT_INDEX = 0;

  public static final double ROTATION_SENSOR_CONE = 0;
}
