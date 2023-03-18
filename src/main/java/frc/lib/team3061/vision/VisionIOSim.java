package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

public class VisionIOSim implements VisionIO {
  private static final String CAMERA_NAME = "simCamera";
  private static final double DIAGONAL_FOV = 70; // FOV in degrees
  private static final int IMG_WIDTH = 1280; // image width in px
  private static final int IMG_HEIGHT = 720; // image heigh in px
  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);

  private Supplier<Pose2d> poseSupplier;
  private SimVisionSystem simVision;
  private AprilTagFieldLayout layout;
  private PhotonPoseEstimator photonPoseEstimator;

  public VisionIOSim(
      AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
    this.layout = layout;
    this.poseSupplier = poseSupplier;

    this.simVision =
        new SimVisionSystem(
            CAMERA_NAME, DIAGONAL_FOV, robotToCamera, 9000, IMG_WIDTH, IMG_HEIGHT, 0);

    this.setLayoutOrigin(OriginPosition.kBlueAllianceWallRightSide);

    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            this.layout, PoseStrategy.MULTI_TAG_PNP, this.camera, robotToCamera);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    this.simVision.processFrame(poseSupplier.get());
    this.photonPoseEstimator.setReferencePose(poseSupplier.get());
    Optional<EstimatedRobotPose> robotPose = this.photonPoseEstimator.update();
    if (robotPose.isPresent()) {
      inputs.hasNewResult = true;
      inputs.lastTimestamp = robotPose.get().timestampSeconds;
      inputs.robotPose = robotPose.get().estimatedPose;
    } else {
      inputs.hasNewResult = false;
    }
  }

  @Override
  public void setLayoutOrigin(OriginPosition origin) {
    layout.setOrigin(origin);
    this.simVision.clearVisionTargets();

    for (AprilTag tag : layout.getTags()) {
      if (layout.getTagPose(tag.ID).isPresent()) {
        this.simVision.addSimVisionTarget(
            new SimVisionTarget(
                layout.getTagPose(tag.ID).get(),
                Units.inchesToMeters(6),
                Units.inchesToMeters(6),
                tag.ID));
      }
    }
  }
}
