package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Intake subsystem hardware interface. */
public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ElevatorIOInputs {
    double extensionSetpointMeters = 0.0;
    double extensionPositionMeters = 0.0;
    double extensionVelocityMetersPerSec = 0.0;
    double extensionClosedLoopErrorMeters = 0.0;
    double extensionAppliedVolts = 0.0;
    double[] extensionCurrentAmps = new double[] {};
    double[] extensionTempCelsius = new double[] {};

    double rotationSetpointRadians = 0.0;
    double rotationPositionRadians = 0.0;
    double rotationVelocityRadiansPerSec = 0.0;
    double rotationClosedLoopErrorRadians = 0.0;
    double rotationAppliedVolts = 0.0;
    double[] rotationCurrentAmps = new double[] {};
    double[] rotationTempCelsius = new double[] {};

    double pitchRadians = 0.0;
    double rollRadians = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the climber open loop at the specified voltage. */
  public default void setExtensionMotorPercentage(double percentage) {}

  /** Run the climber open loop at the specified voltage. */
  public default void setRotationMotorPercentage(double percentage) {}

  public default void setPosition(
      double rotation,
      double extension,
      double rotationExtensionTimeOffset,
      boolean applyTimeOffsetAtStart) {}

  public default boolean isAtSetpoint() {
    return false;
  }

  public default void autoZeroExtension() {}
}
