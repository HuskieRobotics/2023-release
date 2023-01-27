package frc.robot.subsystems.elevator;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.networktables.DoubleSubscriber;

//FIXME ALL CONSTANTS
public class ElevatorConstants {

  /** Make constants for each positions */
  public static final boolean DEBUGGING = true;
  public static final boolean TESTING = true;
  public static final boolean TUNING = true;
  public static final String SUBSYSTEM_NAME = "Elevator";

  public static final int TOP_ROTATION_POSITION = 0;
  public static final int MID_ROTATION_POSITION = 0;
  public static final int LOW_ROTATION_POSITION = 0;
  public static final int CONE_INTAKE_ROTATION_POSITION = 0;
  public static final int CUBE_INTAKE_ROTATION_POSITION = 0;
  public static final int SHELF_ROTATION_POSITION = 0;
  public static final int CHUTE_ROTATION_POSITION = 0;

  public static final int TOP_EXTENSION_POSITION = 0;
  public static final int MID_EXTENSION_POSITION = 0;
  public static final int LOW_EXTENSION_POSITION = 0;
  public static final int CONE_INTAKE_EXTENSION_POSITION = 0;
  public static final int CUBE_INTAKE_EXTENSION_POSITION = 0;
  public static final int SHELF_EXTENSION_POSITION = 0;
  public static final int CHUTE_EXTENSION_POSITION = 0;

  public static final int MIN_EXTENSION_POSITION = 0; 
  public static final int MAX_EXTENSION_POSITION = 0;

  public static final int MIN_ROTATION_POSITION = 0; 
  public static final int MAX_ROTATION_POSITION = 0;

  public static final int ELEVATOR_MOTOR_CAN_ID = 0;
  public static final int ROTATION_ELEVATOR_MOTOR_CAN_ID = 0;

  public static final int EXTENSION_POSITION_PID_F = 0;
  public static final int EXTENSION_POSITION_PID_P = 0;
  public static final int EXTENSION_POSITION_PID_I = 0;
  public static final int EXTENSION_POSITION_PID_D = 0; 
  public static final int EXTENSION_POSITION_PID_I_ZONE = 0;
  public static final int EXTENSION_POSITION_PID_PEAK_OUTPUT = 0;

  public static final int ROTATION_POSITION_PID_F = 0;
  public static final int ROTATION_POSITION_PID_P = 0;
  public static final int ROTATION_POSITION_PID_I = 0;
  public static final int ROTATION_POSITION_PID_D = 0; 
  public static final int ROTATION_POSITION_PID_I_ZONE = 0;
  public static final int ROTATION_POSITION_PID_PEAK_OUTPUT = 0;
  
  public static final double EXTENSION_SLOW_PEAK_OUTPUT = 0.15;             
  public static final double EXTENSION_MAX_ELEVATOR_VELOCITY = 20000; // theoretical maximum 21305
  public static final double EXTENSION_ELEVATOR_ACCELERATION = MAX_ELEVATOR_VELOCITY * 10;
  public static final int EXTENSION_SCURVE_STRENGTH = 0;

  public static final double ROTATION_SLOW_PEAK_OUTPUT = 0.15;     
  public static final double ROTATION_MAX_ELEVATOR_VELOCITY = 20000; // theoretical maximum 21305
  public static final double ROTATION_ELEVATOR_ACCELERATION = MAX_ELEVATOR_VELOCITY * 10;
  public static final int ROTATION_SCURVE_STRENGTH = 0;

  public static final int ELEVATOR_EXTENSION_POSITION_TOLERANCE = 1000;
  public static final double ARBITRARY_FEED_FORWARD_EXTENSION = .02;
  public static final double DEFAULT_EXTENSION_MOTOR_POWER = 0.5;

  public static final int ELEVATOR_ROTATION_POSITION_TOLERANCE = 1000;
  public static final double ARBITRARY_FEED_FORWARD_ROTATION = .02;
  public static final double DEFAULT_ROTATION_MOTOR_POWER = 0.5;

  public static final int TIMEOUT_MS = 30;
  public static final int SLOT_INDEX = 0;
}
