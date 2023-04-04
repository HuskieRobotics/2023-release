package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private VisionIO[] visionIOs;
  private Transform3d[] camerasToRobots;
  private final VisionIOInputs[] ios;
  private double[] lastTimestamps;

  private AprilTagFieldLayout layout;

  private SwerveDrivePoseEstimator poseEstimator;
  private boolean isEnabled = true;
  private boolean isVisionUpdating = false;

  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.WARNING);

  public Vision(VisionIO... visionIO) {
    this.visionIOs = visionIO;
    this.camerasToRobots = RobotConfig.getInstance().getRobotToCameraTransforms();
    this.lastTimestamps = new double[visionIO.length];
    this.ios = new VisionIOInputs[visionIO.length];
    for (int i = 0; i < visionIO.length; i++) {
      this.ios[i] = new VisionIOInputs();
    }

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

  public void updateAlliance(DriverStation.Alliance newAlliance) {

    if (newAlliance == DriverStation.Alliance.Red) {
      layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      for (VisionIO visionIO : visionIOs) {
        visionIO.setLayoutOrigin(OriginPosition.kRedAllianceWallRightSide);
      }
    } else {
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      for (VisionIO visionIO : visionIOs) {
        visionIO.setLayoutOrigin(OriginPosition.kBlueAllianceWallRightSide);
      }
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
    isVisionUpdating = false;
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(ios[i]);
      Logger.getInstance().processInputs("Vision" + i, ios[i]);

      if (lastTimestamps[i] < ios[i].lastTimestamp) {
        lastTimestamps[i] = ios[i].lastTimestamp;
        Pose3d robotPose = getRobotPose(i);

        if (robotPose == null) return;

        if (poseEstimator
                .getEstimatedPosition()
                .minus(robotPose.toPose2d())
                .getTranslation()
                .getNorm()
            < MAX_POSE_DIFFERENCE_METERS) {
          if (isEnabled) {
            poseEstimator.addVisionMeasurement(robotPose.toPose2d(), ios[i].lastTimestamp);
            isVisionUpdating = true;
          }

          Logger.getInstance().recordOutput("Vision/RobotPose" + i, robotPose.toPose2d());
          Logger.getInstance().recordOutput("Vision/isEnabled", isEnabled);
        }
      }
    }
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  private Pose3d getRobotPose(int index) {
    int targetCount = 0;
    Pose3d robotPoseFromClosestTarget = null;
    double closestTargetDistance = Double.MAX_VALUE;

    for (int i = 0; i < 2; i++) {
      Logger.getInstance().recordOutput("Vision/TagPose" + index + "_" + i, new Pose2d());
      Logger.getInstance().recordOutput("Vision/NVRobotPose" + index + "_" + i, new Pose2d());
    }

    for (PhotonTrackedTarget target : ios[index].lastResult.getTargets()) {
      if (isValidTarget(target)) {
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(target.getFiducialId());
        if (tagPoseOptional.isPresent()) {
          Pose3d tagPose = tagPoseOptional.get();
          Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
          Pose3d robotPose = cameraPose.transformBy(camerasToRobots[index].inverse());

          Logger.getInstance()
              .recordOutput("Vision/TagPose" + index + "_" + targetCount, tagPose.toPose2d());
          Logger.getInstance()
              .recordOutput("Vision/NVRobotPose" + index + "_" + targetCount, robotPose.toPose2d());

          double targetDistance =
              target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
          if (targetDistance < VisionConstants.MAX_DISTANCE_TO_TARGET
              && targetDistance < closestTargetDistance) {
            closestTargetDistance = targetDistance;

            robotPoseFromClosestTarget = robotPose;
          }
        }
      }
      targetCount++;
    }

    return robotPoseFromClosestTarget;
  }

  public Pose3d getBestRobotPose() {
    for (int i = 0; i < visionIOs.length; i++) {
      Pose3d robotPose = getRobotPose(i);
      if (robotPose != null) {
        return robotPose;
      }
    }
    return null;
  }

  public void enable(boolean enable) {
    isEnabled = enable;
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY
        && layout.getTagPose(target.getFiducialId()).isPresent();
  }
}
