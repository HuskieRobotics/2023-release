package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  public static class ElevatorIOInputs implements LoggableInputs {
    boolean isControlEnabled = false;

    double leftPosition = 0.0;
    double leftVelocity = 0.0;
    double leftClosedLoopError = 0.0;
    double leftAppliedVolts = 0.0;
    double[] leftCurrentAmps = new double[] {};
    double[] leftTempCelcius = new double[] {};

    double rightPosition = 0.0;
    double rightVelocity = 0.0;
    double rightClosedLoopError = 0.0;
    double rightAppliedVolts = 0.0;
    double[] rightCurrentAmps = new double[] {};
    double[] rightTempCelcius = new double[] {};

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
