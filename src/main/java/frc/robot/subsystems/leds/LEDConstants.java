package frc.robot.subsystems.leds;

public class LEDConstants {
  public enum DriveInfoStates {
    CONE, // YELLOW LED COLOR ON 2nd and 4th sections of a-frame
    CUBE, // PURPLE LED COLOR ON 2nd and 4th sections of a-frame
    TELEOP, // BLUE LED COLOR ON 1st and 3rd sections of a-frame
    AUTO, // ORANGE LED COLOR ON 1st and 3rd sections of a-frame
  }

  // not likely to be used
  public enum RobotStateColors {
    WHITE, // NORMAL OPERATION
    YELLOW, // LOW VOLTAGE
    PURPLE, // AUTO DRIVE DISABLED
    GREEN, // MANIPULATOR SENSOR DISABLED
    BLUE, // VISION FAILUREM, CAMERA NOT FOUND
    ORANGE, // VISION FAILURE, MANUALLY DISABLED VISION
    RED, // LOSS OF CAN DEVICE
    PINK,
    BLINKGREEN,
    BLINKBLUE,
    BLINKPINK,
  }

  public enum RobotErrorStates {
    LOSS_OF_CAN_DEVICE, // RED TOP LEDS
    VISION_FAILURE_DISABLED, // ORANGE TOP LEDS
    LOW_VOLTAGE, // YELLOW TOP LEDS
    MANIPULATOR_SENSOR_DISABLED, // GREEN TOP LEDS
    VISION_FAILURE_NO_CAMERA, // BLUE TOP LEDS
    AUTO_DRIVE_DISABLED, // PURPLE TOP LEDS
  }

  public enum AnimationTypes {
    COLORFLOW,
    FIRE,
    LARSON,
    RAINBOW,
    RGBFADE,
    SINGLEFADE,
    STROBEORANGE,
    STROBEBLUE,
    TWINKLE,
    TWINKLEOFF,
    SETALL,
    YELLOW,
    PURPLE,
    GREEN,
    BLUE,
    ORANGE,
    RED,
  }
}
