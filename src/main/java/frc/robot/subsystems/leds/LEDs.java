package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import java.util.*;

public class LEDs extends SubsystemBase {
  private int nextError, currentError;
  private int timer;
  private boolean cone;
  private boolean auto;
  private final CANdle candle;
  private boolean[] errors;
  private final int ledCount =
      RobotConfig.getInstance().getLEDCount(); // that is how many are there for testing

  // enumerated type of the different types of AnimationTypes
  // not all of these will be used or are that important but it is just good to have a
  // storage of everything
  // FIXME: these need to be changed bases on what aidan needs!

  public enum DriveInfoStates {
    CONE,
    CUBE,
    TELEOP,
    AUTO,
  }

  public enum RobotStateColors {
    WHITE,
    YELLOW,
    PURPLE,
    GREEN,
    BLUE,
    ORANGE,
    RED,
  }

  public enum Errors {
    LOSS_OF_CAN_DEVICE,
    VISION_FAILURE_DISABLED,
    LOW_VOLTAGE,
    MANIPULATOR_SENSOR_DISABLED,
    VISION_FAILURE_NO_CAMERA,
    AUTO_DRIVE_DISABLED,
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

  // animation to be set
  private Animation toAnimate;

  public LEDs() {
    nextError = -1;
    currentError = -1;
    timer = 0;
    cone = true; // temporary
    candle = new CANdle(22, RobotConfig.getInstance().getCANBusName());
    errors = new boolean[6];
    // 0 = loss of can device - red
    // 1 = vision failure, if we disabled vision - orange
    // 2 = low voltage - yellow
    // 3 = manipulator sensor disabled - green
    // 4 = vision failure, if we dont have a camera - blue
    // 5 = auto drive disabled - purple

    CANdleConfiguration configSettings = new CANdleConfiguration();
    configSettings.statusLedOffWhenActive = true;
    configSettings.disableWhenLOS = false;
    configSettings.stripType = LEDStripType.GRB;
    configSettings.brightnessScalar = .1;
    configSettings.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configSettings, 100);
    // changeAnimationTo(AnimationTypes.RAINBOW);
  }

  @Override
  public void periodic() {

    this.checkErrors();

    // setting top values
    if (currentError == 0) { // loss of can device
      this.changeTopStateColor(RobotStateColors.RED);
    } else if (currentError == 1) { // vision failure, we disabled vision
      this.changeTopStateColor(RobotStateColors.ORANGE);
    } else if (currentError == 2) { // low voltage
      this.changeTopStateColor(RobotStateColors.YELLOW);
    } else if (currentError == 3) { // manipulator sensor disabled
      this.changeTopStateColor(RobotStateColors.GREEN);
    } else if (currentError == 4) { // vision failure, we don't have a camera
      this.changeTopStateColor(RobotStateColors.BLUE);
    } else if (currentError == 5) { // auto drive disabled
      this.changeTopStateColor(RobotStateColors.PURPLE);
    } else { // nothing is wrong, default to white
      this.changeTopStateColor(RobotStateColors.WHITE);
    }

    this.configurePickupLEDs();
    this.configureAutoTeleopLEDs();
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

  // two for loops
  // one from current index to end
  // boolean called found next error
  // if not found, then go to next foor loop
  // if at the end of both for loops, next error is still false, then set next error to -1 and white

  public void enableConeLED() {
    cone = true;
  }

  public void enableCubeLED() {
    cone = false;
  }

  public boolean pickupCone() {
    return cone;
  }

  public void enableAutoLED() {
    auto = true;
  }

  public void enableTeleopLED() {
    auto = false;
  }

  public void configureAutoTeleopLEDs() {

    if (auto) {
      this.changeAutoTeleopStateColors(DriveInfoStates.AUTO);
    } else {
      this.changeAutoTeleopStateColors(DriveInfoStates.TELEOP);
    }
  }

  public void configurePickupLEDs() {
    if (cone) {
      this.changePickupStateColors(DriveInfoStates.CONE);
    } else {
      this.changePickupStateColors(DriveInfoStates.CUBE);
    }
  }

  public void setErrorIndex(Errors error, boolean isError) {
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

  public void setLED(int r, int g, int b, int w) {
    candle.setLEDs(r, g, b, w, 0, ledCount);
  }

  public void setLEDAnimation() {
    FireAnimation rainbow = new FireAnimation();
    candle.animate(rainbow);
  }

  /*
   * sets the animation to all the leds to one color
   */
  public void setColors() {
    changeAnimationTo(AnimationTypes.SETALL);
  }

  // Wrappers used for inside the subsystem
  public double getBusVoltage() {
    return candle.getBusVoltage();
  }

  public double get5RailVoltage() {
    return candle.get5VRailVoltage();
  }

  public double getCurrent() {
    return candle.getCurrent();
  }

  public double getTemperature() {
    return candle.getTemperature();
  }

  public void configBrightnessScalar(double percent) {
    candle.configBrightnessScalar(percent, 0);
  }

  public void configLOSBehavior(boolean disableWhenLos) {
    candle.configLOSBehavior(disableWhenLos, 0);
  }

  public void configLedType(LEDStripType type) {
    candle.configLEDType(type, 0);
  }

  public void configStatusLedState(boolean offWhenActive) {
    candle.configStatusLedState(offWhenActive, 0);
  }

  // FIXME: split this into two seperate methods, one for controlling the top and one for
  // controlling side colors

  public void changeTopStateColor(RobotStateColors topColor) {
    switch (topColor) {
      case YELLOW:
        candle.setLEDs(255, 255, 0, 0, 71, 29);
        break;

      case PURPLE:
        candle.setLEDs(255, 0, 255, 0, 71, 29);
        break;

      case GREEN:
        candle.setLEDs(0, 255, 0, 0, 71, 29);
        break;

      case BLUE:
        candle.setLEDs(0, 0, 255, 0, 71, 29);
        break;

      case ORANGE:
        candle.setLEDs(255, 172, 28, 0, 71, 29);
        break;

      case RED:
        candle.setLEDs(255, 0, 0, 0, 71, 29);
        break;

      default:
        candle.setLEDs(255, 255, 255, 0, 71, 29);
        break;
    }
  }

  public void changeAutoTeleopStateColors(DriveInfoStates autoTeleop) {
    switch (autoTeleop) {
      case TELEOP:
        candle.setLEDs(0, 0, 255, 0, 0, 24);
        candle.setLEDs(0, 0, 255, 0, 40, 16);
        candle.setLEDs(0, 0, 255, 0, 116, 16);
        candle.setLEDs(0, 0, 255, 0, 148, 16);
        // other side figure out indexes
        break;
      case AUTO:
        candle.setLEDs(255, 165, 0, 0, 0, 24);
        candle.setLEDs(255, 165, 0, 0, 40, 16);
        candle.setLEDs(255, 165, 0, 0, 116, 16);
        candle.setLEDs(255, 165, 0, 0, 148, 16);
        break;
      default:
        break;
    }
  }

  public void changePickupStateColors(DriveInfoStates coneCube) {
    switch (coneCube) {
      case CONE:
        candle.setLEDs(255, 255, 0, 0, 24, 16);
        candle.setLEDs(255, 255, 0, 0, 56, 15);
        candle.setLEDs(255, 255, 0, 0, 101, 15);
        candle.setLEDs(255, 255, 0, 0, 132, 16);
        break;
      case CUBE:
        candle.setLEDs(255, 0, 255, 0, 24, 16);
        candle.setLEDs(255, 0, 255, 0, 56, 15);
        candle.setLEDs(255, 0, 255, 0, 101, 15);
        candle.setLEDs(255, 0, 255, 0, 132, 16);
        break;
      default:
        break;
    }
  }

  // FIXME: Once Aidan gives the current animations, add them here
  public void changeAnimationTo(AnimationTypes newAnimation) {

    switch (newAnimation) {
      case YELLOW:
        toAnimate = null;
        break;

      case PURPLE:
        toAnimate = null;
        break;

      case GREEN:
        toAnimate = null;
        break;

      case BLUE:
        toAnimate = null;
        break;

      case ORANGE:
        toAnimate = null;
        break;

      case RED:
        toAnimate = null;
        break;

      case COLORFLOW:
        Direction direction = Direction.Forward;
        toAnimate = new ColorFlowAnimation(255, 0, 255, 125, 0.6, ledCount, direction);
        break;

      case FIRE:
        toAnimate = new FireAnimation(0.1, 0.3, ledCount, 0.5, 0.7);
        break;

      case LARSON:
        toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, ledCount, BounceMode.Front, 3);
        break;

      case RAINBOW:
        toAnimate = new RainbowAnimation(1, 0.5, ledCount);
        break;

      case RGBFADE:
        toAnimate = new RgbFadeAnimation(0.7, 0.4, ledCount);
        break;

      case SINGLEFADE:
        toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, ledCount);
        break;

      case STROBEORANGE:
        toAnimate = new StrobeAnimation(255, 69, 0, 0, 98.0 / 256.0, ledCount);
        break;
      case STROBEBLUE:
        toAnimate = new StrobeAnimation(0, 0, 128, 0, 98.0 / 256.0, ledCount);
        break;
      case TWINKLE:
        toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, ledCount, TwinklePercent.Percent6);
        break;

      case TWINKLEOFF:
        toAnimate =
            new TwinkleOffAnimation(70, 90, 175, 0, 0.8, ledCount, TwinkleOffPercent.Percent100);
        break;

      case SETALL:
        toAnimate = null;
        break;
    }
    candle.animate(toAnimate);
  }

  // invoke this within the command that goes to the substation and pass in the elevator
  // isConeOrCube
  public void setConeCubeLED(boolean isCone) {
    // FIXME: Set this to the right RGB color for yellow
    if (isCone) {
      candle.setLEDs(255, 0, 0, 0, 0, 71);
      candle.setLEDs(255, 0, 0, 0, 101, 80);
    } else {
      // FIXME: Set this to the right RGB color for purple
      candle.setLEDs(0, 255, 0, 0, 0, 71);
      candle.setLEDs(0, 255, 0, 0, 101, 80);
    }
  }
}
