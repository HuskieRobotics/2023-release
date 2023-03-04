package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Intake subsystem hardware interface. */
public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class IntakeIOInputs {
    double rotationPositionDeg = 0.0;
    double rotationVelocityDegPerSec = 0.0;
    double rotationClosedLoopError = 0.0;
    double rotationAppliedVolts = 0.0;
    double rotationPower = 0.0;
    double[] rotationCurrentAmps = new double[] {};
    double[] rotationTempCelsius = new double[] {};

    double rollerAppliedVolts = 0.0;
    double[] rollerCurrentAmps = new double[] {};
    double[] rollerTempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake open loop at the specified motor power. */
  public default void setRotationMotorPercentage(double percentage) {}

  /** Run the intake closed loop to the specified position. */
  public default void setRotationPosition(double position, double arbitraryFeedForward) {}

  /** Run the roller open loop at the specified motor power. */
  public default void setRollerMotorPercentage(double percentage) {}

  public default void resetRotationEncoder(double resetPosition) {}
}
