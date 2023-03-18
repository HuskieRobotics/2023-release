package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOPhotonVision implements VisionIO {
  private Alert noCameraConnectedAlert =
      new Alert("specified camera not connected", AlertType.WARNING);
  private final PhotonCamera camera;

  private Supplier<Pose2d> poseSupplier;
  private AprilTagFieldLayout layout;
  private PhotonPoseEstimator photonPoseEstimator;

  public VisionIOPhotonVision(
      String cameraName,
      AprilTagFieldLayout layout,
      Supplier<Pose2d> poseSupplier,
      Transform3d robotToCamera) {
    this.camera = new PhotonCamera(cameraName);
    this.poseSupplier = poseSupplier;
    this.layout = layout;
    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            this.layout, PoseStrategy.MULTI_TAG_PNP, this.camera, robotToCamera);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    this.photonPoseEstimator.setReferencePose(poseSupplier.get());
    Optional<EstimatedRobotPose> robotPose = this.photonPoseEstimator.update();
    if (robotPose.isPresent()) {
      inputs.hasNewResult = true;
      inputs.lastTimestamp = robotPose.get().timestampSeconds;
      inputs.robotPose = robotPose.get().estimatedPose;
    } else {
      inputs.hasNewResult = false;
    }

    noCameraConnectedAlert.set(!camera.isConnected());
  }

  @Override
  public void setLayoutOrigin(OriginPosition origin) {
    layout.setOrigin(origin);
  }
}
