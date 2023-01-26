package frc.robot.subsystems.elevator;

public class ElevatorExtensionConstants {

  /** Make constants for each positions */

  //FIXME extension and rotation setpoints
  public static final int TOP_ROTATION = 0;
  public static final int MID_ROTATION = 0;
  public static final int LOW_ROTATION = 0;

  public static final int TOP_EXTENSION = 0;
  public static final int MID_EXTENSION = 0;
  public static final int LOW_EXTENSION = 0;

  public static final int MIN_HEIGHT = 0; 
  public static final int MAX_HEIGHT = 0;

  public static final int PIGEON_ID = 18;
  public static final int ELEVATOR_MOTOR_CAN_ID = 22;
  public static final int ROTATION_ELEVATOR_MOTOR_CAN_ID = 19;

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
  
  public static final double EXTENSION_SLOW_PEAK_OUTPUT = 0.15;             // FIXME
  public static final double EXTENSION_MAX_ELEVATOR_VELOCITY = 20000; // theoretical maximum 21305
  public static final double EXTENSION_ELEVATOR_ACCELERATION = MAX_ELEVATOR_VELOCITY * 10;
  public static final int EXTENSION_SCURVE_STRENGTH = 0;

  public static final double ROTATION_SLOW_PEAK_OUTPUT = 0.15;             // FIXME
  public static final double ROTATION_MAX_ELEVATOR_VELOCITY = 20000; // theoretical maximum 21305
  public static final double ROTATION_ELEVATOR_ACCELERATION = MAX_ELEVATOR_VELOCITY * 10;
  public static final int ROTATION_SCURVE_STRENGTH = 0;

  public static final int ELEVATOR_EXTENSION_POSITION_TOLERANCE = 1000;
  public static final double ARBITRARY_FEED_FORWARD_EXTENsion = .02;
  public static final double DEFAULT_MOTOR_POWER = 0.5;


  //FIXME FOR rotation
  public static final int ELEVATOR_EXTENSION_POSITION_TOLERANCE = 1000;
  public static final double ARBITRARY_FEED_FORWARD_EXTENsion = .02;
  public static final double DEFAULT_MOTOR_POWER = 0.5;

  public static final int TIMEOUT_MS = 30;
  public static final int SLOT_INDEX = 0;

}
