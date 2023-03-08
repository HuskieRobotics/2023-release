package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.VisionIO.VisionIOInputs;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private VisionIO visionIO;
  private final VisionIOInputs io = new VisionIOInputs();
  private AprilTagFieldLayout layout;

  private double lastTimestamp;
  private SwerveDrivePoseEstimator poseEstimator;
  private boolean isEnabled = true;
  private boolean isVisionUpdating = true;

  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.WARNING);

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();
    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    tabMain
        .addBoolean("isVisionUpdating", () -> isVisionUpdating)
        .withPosition(7, 2)
        .withSize(1, 2);

    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      noAprilTagLayoutAlert.set(false);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      noAprilTagLayoutAlert.set(true);
    }

    for (AprilTag tag : layout.getTags()) {
      Logger.getInstance().recordOutput("Vision/AprilTags/" + tag.ID, tag.pose);
    }
  }

  public double getLatestTimestamp() {
    return io.lastTimestamp;
  }

  public PhotonPipelineResult getLatestResult() {
    return io.lastResult;
  }

  public void updateAlliance(DriverStation.Alliance newAlliance) {
    if (newAlliance == DriverStation.Alliance.Red) {
      layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      visionIO.setLayoutOrigin(OriginPosition.kRedAllianceWallRightSide);
    } else {
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      visionIO.setLayoutOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    for (AprilTag tag : layout.getTags()) {
      if (layout.getTagPose(tag.ID).isPresent()) {
        Logger.getInstance()
            .recordOutput("Vision/AprilTags/" + tag.ID, layout.getTagPose(tag.ID).get());
      }
    }
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(io);
    Logger.getInstance().processInputs("Vision", io);

    if (lastTimestamp < getLatestTimestamp()) {
      lastTimestamp = getLatestTimestamp();
      Pose3d robotPose = getRobotPose();

      if (robotPose == null) return;

      if (poseEstimator
              .getEstimatedPosition()
              .minus(robotPose.toPose2d())
              .getTranslation()
              .getNorm()
          < MAX_POSE_DIFFERENCE_METERS) {
        if (isEnabled) {
          poseEstimator.addVisionMeasurement(robotPose.toPose2d(), getLatestTimestamp());
          isVisionUpdating = true;
        }

        Logger.getInstance().recordOutput("Vision/RobotPose", robotPose.toPose2d());
        Logger.getInstance().recordOutput("Vision/isEnabled", isEnabled);
      }
    }
    isVisionUpdating = false;
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  public Pose3d getRobotPose() {
    for (PhotonTrackedTarget target : getLatestResult().getTargets()) {
      if (isValidTarget(target)) {
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(target.getFiducialId());
        if (tagPoseOptional.isPresent()) {
          Pose3d tagPose = tagPoseOptional.get();
          Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
          Pose3d robotPose =
              cameraPose.transformBy(
                  RobotConfig.getInstance().getRobotToCameraTransform().inverse());
          Logger.getInstance().recordOutput("Vision/NVRobotPose", robotPose.toPose2d());

          return robotPose;
        }
      }
    }
    return null;
  }

  public boolean tagVisible(int id) {
    PhotonPipelineResult result = getLatestResult();
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == id && isValidTarget(target)) {
        return true;
      }
    }
    return false;
  }

  /**
   * returns the best Rotation3d from the robot to the given target.
   *
   * @param id
   * @return the Transform3d or null if there isn't
   */
  public Transform3d getTransform3dToTag(int id) {
    PhotonPipelineResult result = getLatestResult();
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == id && isValidTarget(target)) {
        return RobotConfig.getInstance()
            .getRobotToCameraTransform()
            .plus(target.getBestCameraToTarget());
      }
    }
    return null;
  }

  public Rotation2d getAngleToTag(int id) {
    Transform3d transform = getTransform3dToTag(id);
    if (transform != null) {
      return new Rotation2d(transform.getTranslation().getX(), transform.getTranslation().getY());
    } else {
      return null;
    }
  }

  public double getDistanceToTag(int id) {
    Transform3d transform = getTransform3dToTag(id);
    if (transform != null) {
      return transform.getTranslation().toTranslation2d().getNorm();
    } else {
      return -1;
    }
  }

  public void enable(boolean enable) {
    isEnabled = enable;
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY
        && layout.getTagPose(target.getFiducialId()).isPresent()
        && target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
            < VisionConstants.MAX_DISTANCE_TO_TARGET;
  }
}
