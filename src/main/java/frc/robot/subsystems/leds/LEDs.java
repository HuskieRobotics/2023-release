package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.*;

public class LEDs extends SubsystemBase {

  private final LEDIO io;
  private boolean[] errors;
  private int nextError, currentError;
  private int timer;
  private boolean cone, auto;

  public LEDs(LEDIO io) {
    nextError = -1;
    currentError = -1;
    timer = 0;
    this.io = io;
    ShuffleboardTab tab = Shuffleboard.getTab("LEDs");
    errors = new boolean[6];
    cone = true;
    // auto = false;
  }

  public void enableConeLED() {
    cone = true;
  }

  public void enableCubeLED() {
    cone = false;
  }

  public boolean pickupCone() {
    return cone;
  }

  public boolean autoEnabled() {
    return auto;
  }

  public void enableAutoLED() {
    auto = true;
  }

  public void enableTeleopLED() {
    auto = false;
  }

  @Override
  public void periodic() {
    this.configureAutoTeleopLEDs();
    this.configurePickupLEDs();
    this.configureTopStateLEDs();
    // this.configureErrorLEDs(); will not be implemented
    // Logger.getInstance().processInputs("LEDs", inputs);

  }

  public void checkErrors() {
    boolean foundNextError = false;

    for (int i = 0; i < errors.length; i++) {
      if (errors[i]) {
        if (i > currentError) {
          nextError = i;
          foundNextError = true;
          break;
        }
      }
    }

    if (!foundNextError && currentError != -1) {
      for (int i = 0; i <= currentError; i++) {
        if (errors[i]) {
          nextError = i;
          foundNextError = true;
        }
      }
    }

    if (!foundNextError) {
      nextError = -1;
    }

    if (timer >= 50) {
      currentError = nextError;
      timer = 0;
    } else {
      timer++;
    }
  }

  public void setErrorIndex(RobotErrorStates error, boolean isError) {
    switch (error) {
      case LOSS_OF_CAN_DEVICE:
        errors[0] = isError;
        break;
      case VISION_FAILURE_DISABLED:
        errors[1] = isError;
        break;
      case LOW_VOLTAGE:
        errors[2] = isError;
      case MANIPULATOR_SENSOR_DISABLED:
        errors[3] = isError;
        break;
      case VISION_FAILURE_NO_CAMERA:
        errors[4] = isError;
        break;
      case AUTO_DRIVE_DISABLED:
        errors[5] = isError;
        break;
      default:
        break;
    }
  }

  public void changeAnimationTo(AnimationTypes newAnimation) {
    io.changeAnimationTo(newAnimation);
  }

  public void changePickupStateColors(DriveInfoStates coneCube) {
    io.changePickupStateColors(coneCube);
  }

  public void changeAutoTeleopStateColors(DriveInfoStates autoTeleop) {
    io.changeAutoTeleopStateColors(autoTeleop);
  }

  public void changeTopStateColor(RobotStateColors color) {
    io.changeTopStateColor(color);
  }

  // public void configBrightnessScalar(double percent) {
  //   io.configBrightnessScalar(percent);
  // }

  public void configureAutoTeleopLEDs() {

    // if (DriverStation.isAutonomousEnabled()) {
    //   this.enableAutoLED();
    // } else if (DriverStation.isTeleopEnabled()) {
    //   this.enableTeleopLED();
    // }

    if (this.autoEnabled()) {
      io.changeAutoTeleopStateColors(DriveInfoStates.AUTO);
    } else {
      io.changeAutoTeleopStateColors(DriveInfoStates.TELEOP);
    }
  }

  public void configurePickupLEDs() {
    if (this.pickupCone()) {
      io.changePickupStateColors(DriveInfoStates.CONE);
    } else {
      io.changePickupStateColors(DriveInfoStates.CUBE);
    }
  }

  // not needed until further notice
  // public void configureErrorLEDs() {
  //   if(errors[0]) {
  //     io.changeTopStateColor(RobotStateColors.RED);
  //   } else if (errors[1]) {
  //     io.changeTopStateColor(RobotStateColors.ORANGE);
  //   } else if (errors[2]) {
  //     io.changeTopStateColor(RobotStateColors.YELLOW);
  //   } else if (errors[3]) {
  //     io.changeTopStateColor(RobotStateColors.GREEN);
  //   } else if (errors[4]) {
  //     io.changeTopStateColor(RobotStateColors.BLUE);
  //   } else if (errors[5]) {
  //     io.changeTopStateColor(RobotStateColors.PURPLE);
  //   } else {
  //     io.changeTopStateColor(null);
  //   }
  // }

  public void configureTopStateLEDs() {
    // do we need to configure the top state when it is implemented in the commands already
  }

  public void endMatchLEDs() {
    io.endMatchLEDs();
  }
}
