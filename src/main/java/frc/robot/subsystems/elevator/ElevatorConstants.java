package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.*;
// FIXME ALL CONSTANTS

public class ElevatorConstants {

  /** Make constants for each positions */
  public static final boolean DEBUGGING = true;

  public static final boolean TESTING = true;
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

  // public static final int EXTENSION_POSITION = 0;
  // public static final int ROTATION_POSITION = 0;

  // public static final int TOP_ROTATION_POSITION = 0;
  // public static final int MID_ROTATION_POSITION = 0;
  // public static final int LOW_ROTATION_POSITION = 0;
  // public static final int CONE_INTAKE_ROTATION_POSITION = 0;
  // public static final int CUBE_INTAKE_ROTATION_POSITION = 0;
  // public static final int SHELF_ROTATION_POSITION = 0;
  // public static final int CHUTE_ROTATION_POSITION = 0;

  // public static final int TOP_EXTENSION_POSITION = 0;
  // public static final int MID_EXTENSION_POSITION = 0;
  // public static final int LOW_EXTENSION_POSITION = 0;
  // public static final int CONE_INTAKE_EXTENSION_POSITION = 0;
  // public static final int CUBE_INTAKE_EXTENSION_POSITION = 0;
  // public static final int SHELF_EXTENSION_POSITION = 0;
  // public static final int CHUTE_EXTENSION_POSITION = 0;

  public static final double TOP_ROTATION_POSITION_TIME = 0;
  public static final double MID_ROTATION_POSITION_TIME = 0;
  public static final double LOW_ROTATION_POSITION_TIME = 0;
  public static final double CONE_INTAKE_ROTATION_POSITION_TIME = 0;
  public static final double CUBE_INTAKE_ROTATION_POSITION_TIME = 0;
  public static final double SHELF_ROTATION_POSITION_TIME = 0;
  public static final double CHUTE_ROTATION_POSITION_TIME = 0;

  public static final double TOP_EXTENSION_POSITION_TIME = 0;
  public static final double MID_EXTENSION_POSITION_TIME = 0;
  public static final double LOW_EXTENSION_POSITION_TIME = 0;
  public static final double CONE_INTAKE_EXTENSION_POSITION_TIME = 0;
  public static final double CUBE_INTAKE_EXTENSION_POSITION_TIME = 0;
  public static final double SHELF_EXTENSION_POSITION_TIME = 0;
  public static final double CHUTE_EXTENSION_POSITION_TIME = 0;

  public static final double MIN_EXTENSION_POSITION = 0.0;
  public static final double MAX_EXTENSION_POSITION = 1.722;

  // FIXME: change to starting position when holding a cone
  public static final int START_EXTENSION_POSITION_INCHES = 0;

  public static final double MIN_ROTATION_POSITION = 0.0;
  public static final double MAX_ROTATION_POSITION = 1.297;

  public static final int EXTENSION_ELEVATOR_MOTOR_CAN_ID = 5;
  public static final int ROTATION_ELEVATOR_MOTOR_CAN_ID = 19;

  public static final boolean EXTENSION_INVERTED = false;
  public static final double EXTENSION_POSITION_PID_F = 0;
  public static final double EXTENSION_POSITION_PID_P = 0.75;
  public static final double EXTENSION_POSITION_PID_I = 0;
  public static final double EXTENSION_POSITION_PID_D = 0;
  public static final double EXTENSION_POSITION_PID_I_ZONE = 0;
  public static final double EXTENSION_POSITION_PID_PEAK_OUTPUT = 0.5;
  public static final double EXTENSION_MAX_STALL_POSITION_OFFSET_METERS = 0.05;
  public static final double EXTENSION_MAX_STALL_VELOCITY_METERS_PER_SECOND = 0.01;

  public static final boolean ROTATION_INVERTED = false;
  public static final double ROTATION_POSITION_PID_F = 0;
  public static final double ROTATION_POSITION_PID_P = 1.0;
  public static final double ROTATION_POSITION_PID_I = 0.01;
  public static final double ROTATION_POSITION_PID_D = 0;
  public static final double ROTATION_POSITION_PID_I_ZONE = 0.1;
  public static final double ROTATION_POSITION_PID_PEAK_OUTPUT = 1.0;

  public static final int PIGEON_ID = 4;
  public static final double EXTENSION_SLOW_PEAK_OUTPUT = 0.15;
  public static final double EXTENSION_MAX_ELEVATOR_VELOCITY = 20000; // theoretical maximum 21305
  public static final double EXTENSION_ELEVATOR_ACCELERATION = EXTENSION_MAX_ELEVATOR_VELOCITY * 10;
  public static final double EXTENSION_SCURVE_STRENGTH = 0;

  public static final double ROTATION_SLOW_PEAK_OUTPUT = 0.15;
  public static final double ROTATION_MAX_ELEVATOR_VELOCITY = 20000; // theoretical maximum 21305
  public static final double ROTATION_ELEVATOR_ACCELERATION = ROTATION_MAX_ELEVATOR_VELOCITY * 10;
  public static final double ROTATION_SCURVE_STRENGTH = 0;

  public static final double ELEVATOR_EXTENSION_POSITION_TOLERANCE = .02;
  public static final double DEFAULT_EXTENSION_MOTOR_POWER = 0.5;
  public static final double MAX_MANUAL_POWER_EXTENSION = .2;

  public static final double ELEVATOR_ROTATION_POSITION_TOLERANCE = .01;
  public static final double DEFAULT_ROTATION_MOTOR_POWER = 0.5;
  public static final double MAX_MANUAL_POWER_ROTATION = .2;

  public static final double EXTENSION_PULLEY_CIRCUMFERENCE = Units.inchesToMeters(1.128) * Math.PI;
  public static final double EXTENSION_GEAR_RATIO = 3.0;

  public static final int PIGEON_UNITS_PER_ROTATION = 8192;

  public static final int TIMEOUT_MS = 30;
  public static final int SLOT_INDEX = 0;

  public static final double ROTATION_SENSOR_CONE = 0;
}
