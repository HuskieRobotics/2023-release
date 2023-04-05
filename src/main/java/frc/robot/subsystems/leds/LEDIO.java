package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.LEDConstants.*;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {

  public default void changeTopStateColor(RobotStateColors color) {}

  public default void changeAnimationTo(AnimationTypes newAnimation) {}

  public default void changePickupStateColors(DriveInfoStates coneCube) {}

  public default void changeAutoTeleopStateColors(DriveInfoStates autoTeleop) {}

  public default void clearAnimation() {}

  public default void endMatchLEDs() {}

  // public default void configBrightnessScalar(double percent) {}
}
