package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    boolean hasNewResult = false;
    double lastTimestamp = 0.0;
    Pose3d robotPose = new Pose3d();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setLayoutOrigin(OriginPosition origin) {}
}
