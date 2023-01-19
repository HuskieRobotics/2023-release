package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface ElevatorIOInputs {
  /** Contains all of the input data received from hardware. */
  public static class ElevatorIO implements LoggableInputs {
    boolean isControlEnabled = false;

    double angledPosition = 0.0;
    double angledVelocity = 0.0;
    double angledClosedLoopError = 0.0;
    double angledAppliedVolts = 0.0;
    double[] angledCurrentAmps = new double[] {};
    double[] angledTempCelcius = new double[] {};

    double extendPosition = 0.0;
    double extendVelocity = 0.0;
    double extendClosedLoopError = 0.0;
    double extendAppliedVolts = 0.0;
    double[] extendCurrentAmps = new double[] {};
    double[] extendTempCelcius = new double[] {};

    double pitch = 0.0;

    public void toLog(LogTable table) {
     
    }

    public void fromLog(LogTable table) {
      isControlEnabled = table.getBoolean("ControlEnabled", isControlEnabled);

    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Enable/Disable collector. */
  public default void setControlEnabled(boolean controlEnabled) {}

  /** Run the climber open loop at the specified voltage. */
  public default void setMotorPercentage(double percentage) {}

  /** Run the climber closed loop to the specified position. */
  public default void setPosition(double position, double arbitraryFeedForward) {}

  /** Set position feed forward constant. */
  public default void configureKF(double kF) {}

  /** Set position proportional constant. */
  public default void configureKP(double kP) {}

  /** Set position integrative constant. */
  public default void configureKI(double kI) {}

  /** Set position derivative constant. */
  public default void configureKD(double kD) {}

  /** Set closed loop peak output. */
  public default void configClosedLoopPeakOutput(double peakOutput) {}
}
