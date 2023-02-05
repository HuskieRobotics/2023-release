package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Intake subsystem hardware interface. */
public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ElevatorIOInputs {
    boolean isControlEnabled = false;

    double extensionSetpointMeters = 0.0;
    double extensionPositionMeters = 0.0;
    double extensionVelocityMetersPerSec = 0.0;
    double extensionClosedLoopError = 0.0;
    double extensionAppliedVolts = 0.0;
    double[] extensionCurrentAmps = new double[] {};
    double[] extensionTempCelsius = new double[] {};

    double rotationSetpointRadians = 0.0;
    double rotationPositionRadians = 0.0;
    double rotationVelocityRadiansPerSec = 0.0;
    double rotationClosedLoopError = 0.0;
    double rotationAppliedVolts = 0.0;
    double[] rotationCurrentAmps = new double[] {};
    double[] rotationTempCelsius = new double[] {};

    double pitch = 0.0;
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
