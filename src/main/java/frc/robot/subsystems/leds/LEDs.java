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

public class LEDs extends SubsystemBase {
  private final CANdle candle;
  private final int ledCount =
      RobotConfig.getInstance().getLedCount(); // that is how many are there for testing

  // enumerated type of the different types of AnimationTypes
  // not all of these will be used or are that important but it is just good to have a
  // storage of everything
  // FIXME: these need to be changed bases on what aidan needs!
  public enum AnimationTypes {
    COLORFLOW,
    FIRE,
    LARSON,
    RAINBOW,
    RGBFADE,
    SINGLEFADE,
    STROBE,
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
    candle = new CANdle(22, RobotConfig.getInstance().getCANBusName());

    CANdleConfiguration configSettings = new CANdleConfiguration();
    configSettings.statusLedOffWhenActive = true;
    configSettings.disableWhenLOS = false;
    configSettings.stripType = LEDStripType.GRB;
    configSettings.brightnessScalar = .1;
    configSettings.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configSettings, 100);
    changeAnimationTo(AnimationTypes.RAINBOW);
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

      case STROBE:
        toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, ledCount);
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
}
