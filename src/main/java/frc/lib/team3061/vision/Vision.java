package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import java.io.IOException;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private VisionIO[] visionIOs;
  private Transform3d[] camerasToRobots;
  private final VisionIOInputsAutoLogged[] ios;
  private double[] lastTimestamps;

  private AprilTagFieldLayout layout;

  private SwerveDrivePoseEstimator poseEstimator;
  private boolean isEnabled = true;
  private boolean isVisionUpdating = false;
  private Pose3d lastRobotPose = new Pose3d();

  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.WARNING);

  public Vision(VisionIO... visionIO) {
    this.visionIOs = visionIO;
    this.camerasToRobots = RobotConfig.getInstance().getRobotToCameraTransforms();
    this.lastTimestamps = new double[visionIO.length];
    this.ios = new VisionIOInputsAutoLogged[visionIO.length];
    for (int i = 0; i < visionIO.length; i++) {
      this.ios[i] = new VisionIOInputsAutoLogged();
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
    double mostRecentTimestamp = 0.0;
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(ios[i]);
      Logger.getInstance().processInputs("Vision" + i, ios[i]);

      if (lastTimestamps[i] < ios[i].lastTimestamp) {
        lastTimestamps[i] = ios[i].lastTimestamp;

        if (ios[i].lastTimestamp > mostRecentTimestamp) {
          mostRecentTimestamp = ios[i].lastTimestamp;
          lastRobotPose = ios[i].robotPose;
        }

        Logger.getInstance().recordOutput("Vision/NVRobotPose" + i, ios[i].robotPose.toPose2d());

        if (poseEstimator
                .getEstimatedPosition()
                .minus(ios[i].robotPose.toPose2d())
                .getTranslation()
                .getNorm()
            < MAX_POSE_DIFFERENCE_METERS) {
          if (isEnabled) {
            poseEstimator.addVisionMeasurement(ios[i].robotPose.toPose2d(), ios[i].lastTimestamp);
            isVisionUpdating = true;
          }

          Logger.getInstance().recordOutput("Vision/RobotPose" + i, ios[i].robotPose.toPose2d());
          Logger.getInstance().recordOutput("Vision/isEnabled", isEnabled);
        }
      }
    }
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  public Pose3d getBestRobotPose() {
    if (isVisionUpdating) {
      return lastRobotPose;
    } else {
      return null;
    }
  }

  public void enable(boolean enable) {
    isEnabled = enable;
  }
}
