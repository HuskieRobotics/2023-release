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
import frc.lib.team3061.RobotConfig;
import frc.robot.subsystems.leds.LEDConstants.*;
import java.util.*;

public class LEDIOCANdle implements LEDIO {
  private final CANdle candle;
  private final int ledCount =
      RobotConfig.getInstance().getLEDCount(); // that is how many are there for testing

  // enumerated type of the different types of AnimationTypes
  // not all of these will be used or are that important but it is just good to have a
  // storage of everything
  // FIXME: these need to be changed bases on what aidan needs!

  // animation to be set
  private Animation toAnimate;

  public LEDIOCANdle() {
    candle = new CANdle(22, RobotConfig.getInstance().getCANBusName());
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

  // two for loops
  // one from current index to end
  // boolean called found next error
  // if not found, then go to next foor loop
  // if at the end of both for loops, next error is still false, then set next error to -1 and white

  // public void setLEDAnimation() {
  // FireAnimation rainbow = new FireAnimation();
  // candle.animate(rainbow);
  // }

  // Wrappers used for inside the subsystem
  /* unused configuration or getter methods just in case
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

  public void configLOSBehavior(boolean disableWhenLos) {
    candle.configLOSBehavior(disableWhenLos, 0);
  }

  public void configLedType(LEDStripType type) {
    candle.configLEDType(type, 0);
  }

  public void configStatusLedState(boolean offWhenActive) {
    candle.configStatusLedState(offWhenActive, 0);
  }
  */

  public void configBrightnessScalar(double percent) {
    candle.configBrightnessScalar(percent, 0);
  }

  public void clearAnimation() {
    candle.clearAnimation(0);
  }

  public void changeTopStateColor(RobotStateColors color) {
    this.clearAnimation();
    switch (color) {
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

      case BLINKGREEN:
        candle.clearAnimation(0);
        candle.animate(new StrobeAnimation(0, 2655, 0, 0, 98.0 / 256.0, 29, 72));
        break;
      case BLINKPINK:
        candle.clearAnimation(0);
        candle.animate(new StrobeAnimation(159, 43, 104, 0, 98.0 / 256.0, 29, 72));
        break;
      case BLINKBLUE:
        candle.animate(new StrobeAnimation(0, 0, 255, 0, 98.0 / 256.0, 29, 72));
        break;
      case PINK:
        candle.clearAnimation(0);
        candle.setLEDs(159, 43, 104, 0, 71, 29);
      case WHITE:
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0, 100, 71, 29);
        break;
      default:
        candle.setLEDs(255, 255, 255, 0, 71, 29);
        break;
    }
  }

  public void changeAutoTeleopStateColors(DriveInfoStates autoTeleop) {
    switch (autoTeleop) {
      case TELEOP:
        candle.setLEDs(0, 191, 255, 0, 0, 24);
        candle.setLEDs(0, 191, 255, 0, 40, 16);
        candle.setLEDs(0, 191, 255, 0, 116, 16);
        candle.setLEDs(0, 191, 255, 0, 148, 16);
        break;
      case AUTO:
        candle.setLEDs(255, 69, 0, 0, 0, 24);
        candle.setLEDs(255, 69, 0, 0, 40, 16);
        candle.setLEDs(255, 69, 0, 0, 116, 16);
        candle.setLEDs(255, 69, 0, 0, 148, 16);
        break;
      default:
        break;
    }
  }

  public void changePickupStateColors(DriveInfoStates coneCube) {
    switch (coneCube) {
      case CONE_SHELF:
        candle.setLEDs(255, 215, 0, 0, 24, 16);
        candle.setLEDs(255, 215, 0, 0, 56, 15);
        candle.setLEDs(255, 215, 0, 0, 101, 15);
        candle.setLEDs(255, 215, 0, 0, 132, 16);
        break;
      case CUBE_SHELF:
        candle.setLEDs(75, 0, 130, 0, 24, 16);
        candle.setLEDs(75, 0, 130, 0, 56, 15);
        candle.setLEDs(75, 0, 130, 0, 101, 15);
        candle.setLEDs(75, 0, 130, 0, 132, 16);
        break;
      case CUBE_CHUTE:
        candle.setLEDs(255, 0, 0, 0, 24, 16);
        candle.setLEDs(255, 0, 0, 0, 56, 15);
        candle.setLEDs(255, 0, 0, 0, 101, 15);
        candle.setLEDs(255, 0, 0, 0, 132, 16);
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

  public void endMatchLEDs() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 255, 0, 24, 16);
    candle.setLEDs(0, 0, 255, 0, 56, 15);
    candle.setLEDs(0, 0, 255, 0, 101, 15);
    candle.setLEDs(0, 0, 255, 0, 132, 16);

    candle.setLEDs(255, 69, 0, 0, 0, 24);
    candle.setLEDs(255, 69, 0, 0, 40, 16);
    candle.setLEDs(255, 69, 0, 0, 116, 16);
    candle.setLEDs(255, 69, 0, 0, 148, 16);
  }
}
