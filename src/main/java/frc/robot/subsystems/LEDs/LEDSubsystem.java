package frc.robot.subsystems.LEDs;
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
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle m_candle;
  private final int LED_COUNT = 210; // that is how many are there for testing
  private CommandXboxController m_Controller;

  // initiation of what the animation will be (switch later)
  private Animation m_toAnimate = null;

  // enumerated type of the different types of AnimationTypes
  // not all of these will be used or are that important but it is just good to have a
  // storage of everything
  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll,
    Yellow,
    Purple,
    Green,
    Blue,
    Orange,
    Red,
  }

  private AnimationTypes m_currentAnimation;

  // if we use an xbox controller as a tester
  public LEDSubsystem(CommandXboxController controller) {
    m_candle = new CANdle(25);
    this.m_Controller = controller;
    changeAnimationTo(AnimationTypes.SetAll);

    CANdleConfiguration configSettings = new CANdleConfiguration();
    configSettings.statusLedOffWhenActive = true;
    configSettings.disableWhenLOS = false;
    configSettings.stripType = LEDStripType.GRB;
    configSettings.brightnessScalar = 0.1;
    configSettings.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configSettings, 100);
  }

  public LEDSubsystem() {
    m_candle = new CANdle(25);
    // this.m_Controller = null;
    changeAnimationTo(AnimationTypes.SetAll);

    CANdleConfiguration configSettings = new CANdleConfiguration();
    configSettings.statusLedOffWhenActive = true;
    configSettings.disableWhenLOS = false;
    configSettings.stripType = LEDStripType.GRB;
    configSettings.brightnessScalar = 0.1;
    // configSettings.brightnessScalar = 0.1;
    configSettings.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configSettings, 100);
  }

  public void setLED(int r, int g, int b, int w) {
    m_candle.setLEDs(r, g, b, w, 0, 27);
    System.out.println("Bus Voltage: " + m_candle.getBusVoltage());
    System.out.println("5V Voltage: " + m_candle.get5VRailVoltage());
  }

  public void setLEDAnimation(){
    FireAnimation rainbow = new FireAnimation();
    m_candle.animate(rainbow);
  }

  /** Increments the animation by one more in the enum type */
  public void increaseAnimation() {
    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimationTo(AnimationTypes.Fire);
        break;
      case Fire:
        changeAnimationTo(AnimationTypes.Larson);
        break;
      case Larson:
        changeAnimationTo(AnimationTypes.Rainbow);
        break;
      case Rainbow:
        changeAnimationTo(AnimationTypes.RgbFade);
        break;
      case RgbFade:
        changeAnimationTo(AnimationTypes.SingleFade);
        break;
      case SingleFade:
        changeAnimationTo(AnimationTypes.Strobe);
        break;
      case Strobe:
        changeAnimationTo(AnimationTypes.Twinkle);
        break;
      case Twinkle:
        changeAnimationTo(AnimationTypes.TwinkleOff);
        break;
      case TwinkleOff:
        changeAnimationTo(AnimationTypes.ColorFlow);
        break;
      case SetAll:
        changeAnimationTo(AnimationTypes.ColorFlow);
        break;
    }
  }

  /** Decrements the animation to the one before in the enum type */
  public void decreaseAnimation() {
    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimationTo(AnimationTypes.TwinkleOff);
        break;
      case Fire:
        changeAnimationTo(AnimationTypes.ColorFlow);
        break;
      case Larson:
        changeAnimationTo(AnimationTypes.Fire);
        break;
      case Rainbow:
        changeAnimationTo(AnimationTypes.Larson);
        break;
      case RgbFade:
        changeAnimationTo(AnimationTypes.Rainbow);
        break;
      case SingleFade:
        changeAnimationTo(AnimationTypes.RgbFade);
        break;
      case Strobe:
        changeAnimationTo(AnimationTypes.SingleFade);
        break;
      case Twinkle:
        changeAnimationTo(AnimationTypes.Strobe);
        break;
      case TwinkleOff:
        changeAnimationTo(AnimationTypes.Twinkle);
        break;
      case SetAll:
        changeAnimationTo(AnimationTypes.ColorFlow);
        break;
    }
  }

  /*
   * sets the animation to all the leds to one color
   */
  public void setColors() {
    changeAnimationTo(AnimationTypes.SetAll);
  }

  // Wrappers used for inside the subsystem
  public double getBusVoltage() {
    return m_candle.getBusVoltage();
  }

  public double get5RailVoltage() {
    return m_candle.get5VRailVoltage();
  }

  public double getCurrent() {
    return m_candle.getCurrent();
  }

  public double getTemperature() {
    return m_candle.getTemperature();
  }

  public void configBrightnessScalar(double percent) {
    m_candle.configBrightnessScalar(percent, 0);
  }

  public void configLOSBehavior(boolean disableWhenLos) {
    m_candle.configLOSBehavior(disableWhenLos, 0);
  }

  public void configLedType(LEDStripType type) {
    m_candle.configLEDType(type, 0);
  }

  public void configStatusLedState(boolean offWhenActive) {
    m_candle.configStatusLedState(offWhenActive, 0);
  }

  public void changeAnimationTo(AnimationTypes newAnimation) {
    m_currentAnimation = newAnimation;

    switch (newAnimation) {
      case Yellow:
        m_toAnimate = null; 
        break;
      
      case Purple:
        m_toAnimate = null; 
        break;

      case Green:
        m_toAnimate = null; 
        break;

      case Blue:
        m_toAnimate = null; 
        break;
      
      case Orange:
        m_toAnimate = null; 
        break;

      case Red:
        m_toAnimate = null; 
        break;
      
      case ColorFlow:
        Direction direction = Direction.Forward;
        m_toAnimate = new ColorFlowAnimation(255, 0, 255, 125, 0.6, LED_COUNT, direction);
        break;

      case Fire:
        m_toAnimate = new FireAnimation(0.1, 0.3, LED_COUNT, 0.5, 0.7);
        break;

      case Larson:
        m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LED_COUNT, BounceMode.Front, 3);
        break;

      case Rainbow:
        m_toAnimate = new RainbowAnimation(0.1, 0.1, LED_COUNT);
        break;

      case RgbFade:
        m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LED_COUNT);
        break;

      case SingleFade:
        m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LED_COUNT);
        break;

      case Strobe:
        m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LED_COUNT);
        break;

      case Twinkle:
        m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LED_COUNT, TwinklePercent.Percent6);
        break;

      case TwinkleOff:
        m_toAnimate =
            new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LED_COUNT, TwinkleOffPercent.Percent100);
        break;

      case SetAll:
        m_toAnimate = null;
        break;
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_toAnimate == null){
     switch (m_currentAnimation) {
      case Yellow:
        m_candle.setLEDs(255, 255, 0);
        break;
      
      case Purple:
        m_candle.setLEDs(255, 0, 255);
        break;

      case Green:
        m_candle.setLEDs(0, 255, 0);
        break;

      case Blue:
        m_candle.setLEDs(0, 0, 255);
        break;
      
      case Orange:
        m_candle.setLEDs(255, 165, 0);
        break;

      case Red:
        m_candle.setLEDs(255, 0, 0);
        break;
     }
    } else {
      m_candle.animate(m_toAnimate); 
    }

  }


}
