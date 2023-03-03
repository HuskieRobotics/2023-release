package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
  @AutoLog
  public static class ManipulatorIOInputs {
    double positionDeg = 0.0;
    double appliedPercentage = 0.0;
    double[] statorCurrentAmps = new double[] {};
    boolean isBlocked = false;
    boolean isOpen = false;
    boolean isClosed = false;
    boolean wasZeroed = false;
    boolean isCanDevicePresent = false;
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setPower(double percentage) {}

  public default boolean isCanDevicePresent() {
    return false;
  }
}
