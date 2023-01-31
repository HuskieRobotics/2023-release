package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Intake subsystem hardware interface. */
public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  public static class ElevatorIOInputs implements LoggableInputs {
    boolean isControlEnabled = false;
    
    double extensionPosition = 0.0;
    double extensionVelocity = 0.0;
    double extensionClosedLoopError = 0.0;
    double extensionAppliedVolts = 0.0;
    double[] extensionCurrentAmps = new double[] {};
    double[] extensionTempCelcius = new double[] {};

    double rotationPosition = 0.0;
    double rotationVelocity = 0.0;
    double rotationClosedLoopError = 0.0;
    double rotationAppliedVolts = 0.0;
    double[] rotationCurrentAmps = new double[] {};
    double[] rotationTempCelcius = new double[] {};

    double pitch = 0.0;
    public void toLog(LogTable table) {
     
    }

    public void fromLog(LogTable table) {
      isControlEnabled = table.getBoolean("ControlEnabled", isControlEnabled);

    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the climber open loop at the specified voltage. */
  public default void setExtensionMotorPercentage(double percentage) {}

  /** Run the climber closed loop to the specified position. */
  public default void setExtensionPosition(double position, double arbitraryFeedForward) {}

  /** Set position feed forward constant. */
  public default void configureExtensionKF(double kF) {}

  /** Set position proportional constant. */
  public default void configureExtensionKP(double kP) {}

  /** Set position integrative constant. */
  public default void configureExtensionKI(double kI) {}

  /** Set position derivative constant. */
  public default void configureExtensionKD(double kD) {}

  /** Set closed loop peak output. */
  public default void configExtensionClosedLoopPeakOutput(double peakOutput) {}

  /** Run the climber open loop at the specified voltage. */
  public default void setRotationMotorPercentage(double percentage) {}

  /** Run the climber closed loop to the specified position. */
  public default void setRotationPosition(double position, double arbitraryFeedForward) {}
  
  /** Set position feed forward constant. */
  public default void configureRotationKF(double kF) {}
  
  /** Set position proportional constant. */
  public default void configureRotationKP(double kP) {}
  
  /** Set position integrative constant. */
  public default void configureRotationKI(double kI) {}
  
  /** Set position derivative constant. */
  public default void configureRotationKD(double kD) {}
  
  /** Set closed loop peak output. */
  public default void configRotationClosedLoopPeakOutput(double peakOutput) {}

  public default void setControlEnabled(boolean controlEnabled) {}

}
