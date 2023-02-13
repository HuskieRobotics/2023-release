package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.team3061.RobotConfig;

/**
 * The FieldConstants class contains various constants relating to the FRC Game Arena for the
 * Region2d and Field2d classes.
 */
public final class FieldConstants {
  // Current FieldConstants are for the FRC 2023 Game "Charged Up"
  // NOTE TO SELF: All constants to be accessed globally should be initialized using (public static)

  public static final double FIELD_WIDTH_METERS = 8.0137;

  // Points making up the Community Zone (not including Charging Station)
  public static final Translation2d COMMUNITY_POINT_1 = new Translation2d(0, 0);
  public static final Translation2d COMMUNITY_POINT_2 = new Translation2d(0, 5.49);
  public static final Translation2d COMMUNITY_POINT_3 = new Translation2d(3, 5.49);
  public static final Translation2d COMMUNITY_POINT_4 = new Translation2d(4.91, 5.49);
  public static final Translation2d COMMUNITY_POINT_5 = new Translation2d(4.91, 3.98);
  public static final Translation2d COMMUNITY_POINT_6 = new Translation2d(3, 3.98);
  public static final Translation2d COMMUNITY_POINT_7 = new Translation2d(3, 1.51);
  public static final Translation2d COMMUNITY_POINT_8 = new Translation2d(4.91, 1.51);
  public static final Translation2d COMMUNITY_POINT_9 = new Translation2d(4.91, 0);
  public static final Translation2d COMMUNITY_POINT_10 = new Translation2d(3, 0);

  // Points making up the Loading Zone
  public static final Translation2d LOADING_ZONE_POINT_1 = new Translation2d(16.54, 5.49);
  public static final Translation2d LOADING_ZONE_POINT_2 = new Translation2d(16.54, 8.02);
  public static final Translation2d LOADING_ZONE_POINT_3 = new Translation2d(13.18, 8.02);
  public static final Translation2d LOADING_ZONE_POINT_4 = new Translation2d(9.83, 8.02);
  public static final Translation2d LOADING_ZONE_POINT_5 = new Translation2d(9.83, 5.49);
  public static final Translation2d LOADING_ZONE_POINT_6 = new Translation2d(13.18, 5.49);

  // Points making up the rest of the Game Field (not including opposite Alliance zones)
  public static final Translation2d FIELD_POINT_1 = new Translation2d(4.91, 0);
  public static final Translation2d FIELD_POINT_2 = new Translation2d(4.91, 5.49);
  public static final Translation2d FIELD_POINT_3 = new Translation2d(11.63, 5.49);
  public static final Translation2d FIELD_POINT_4 = new Translation2d(11.63, 0);
  public static final Translation2d FIELD_POINT_5 = new Translation2d(6.71, 5.49);
  public static final Translation2d FIELD_POINT_6 = new Translation2d(6.71, 8.02);
  public static final Translation2d FIELD_POINT_7 = new Translation2d(9.83, 8.02);
  public static final Translation2d FIELD_POINT_8 = new Translation2d(9.83, 5.49);

  public static final Translation2d[] COMMUNITY_REGION_POINTS_1 =
      new Translation2d[] {
        COMMUNITY_POINT_1, COMMUNITY_POINT_2, COMMUNITY_POINT_3, COMMUNITY_POINT_10
      };
  public static final Translation2d[] COMMUNITY_REGION_POINTS_2 =
      new Translation2d[] {
        COMMUNITY_POINT_6, COMMUNITY_POINT_3, COMMUNITY_POINT_4, COMMUNITY_POINT_5
      };
  public static final Translation2d[] COMMUNITY_REGION_POINTS_3 =
      new Translation2d[] {
        COMMUNITY_POINT_10, COMMUNITY_POINT_7, COMMUNITY_POINT_8, COMMUNITY_POINT_9
      };

  public static final Translation2d[] LOADING_ZONE_REGION_POINTS_1 =
      new Translation2d[] {
        LOADING_ZONE_POINT_1, LOADING_ZONE_POINT_2, LOADING_ZONE_POINT_3, LOADING_ZONE_POINT_6
      };
  public static final Translation2d[] LOADING_ZONE_REGION_POINTS_2 =
      new Translation2d[] {
        LOADING_ZONE_POINT_3, LOADING_ZONE_POINT_4, LOADING_ZONE_POINT_5, LOADING_ZONE_POINT_6
      };

  public static final Translation2d[] FIELD_ZONE_REGION_POINTS_1 =
      new Translation2d[] {FIELD_POINT_1, FIELD_POINT_2, FIELD_POINT_3, FIELD_POINT_4};
  public static final Translation2d[] FIELD_ZONE_REGION_POINTS_2 =
      new Translation2d[] {FIELD_POINT_5, FIELD_POINT_6, FIELD_POINT_7, FIELD_POINT_8};

  public static final Region2d COMMUNITY_REGION_1 = new Region2d(COMMUNITY_REGION_POINTS_1);
  public static final Region2d COMMUNITY_REGION_2 = new Region2d(COMMUNITY_REGION_POINTS_2);
  public static final Region2d COMMUNITY_REGION_3 = new Region2d(COMMUNITY_REGION_POINTS_3);

  public static final Region2d LOADING_ZONE_REGION_1 = new Region2d(LOADING_ZONE_REGION_POINTS_1);
  public static final Region2d LOADING_ZONE_REGION_2 = new Region2d(LOADING_ZONE_REGION_POINTS_2);

  public static final Region2d FIELD_ZONE_REGION_1 = new Region2d(FIELD_ZONE_REGION_POINTS_1);
  public static final Region2d FIELD_ZONE_REGION_2 = new Region2d(FIELD_ZONE_REGION_POINTS_2);

  public static final Translation2d REGION_1_2_TRANSITION_POINT =
      new Translation2d(3.5, (5.49 + 3.98) / 2.0);
  public static final Translation2d REGION_2_1_TRANSITION_POINT =
      new Translation2d(2.5, (5.49 + 3.98) / 2.0);
  public static final Translation2d REGION_1_3_TRANSITION_POINT =
      new Translation2d(3.5, (1.51 / 2.0));
  public static final Translation2d REGION_3_1_TRANSITION_POINT =
      new Translation2d(2.5, (1.51 / 2.0));

  // TODO: Merge the Field2ds into a big Field2d to allow for pathing between the Regions within the
  // Fields
  public static final Field2d COMMUNITY_ZONE =
      new Field2d(new Region2d[] {COMMUNITY_REGION_1, COMMUNITY_REGION_2, COMMUNITY_REGION_3});
  public static final Field2d LOADING_ZONE =
      new Field2d(new Region2d[] {LOADING_ZONE_REGION_1, LOADING_ZONE_REGION_2});
  public static final Field2d FIELD_ZONE =
      new Field2d(new Region2d[] {FIELD_ZONE_REGION_1, FIELD_ZONE_REGION_2});

  // Grid Length: ~ 1.38m
  public static final Pose2d GRID_1_NODE_1 =
      new Pose2d(
          1.38 + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 + 0.03,
          0.51,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_1_NODE_2 =
      new Pose2d(
          1.38 + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 + 0.03,
          1.05,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_1_NODE_3 =
      new Pose2d(
          1.38 + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 + 0.03,
          1.63,
          Rotation2d.fromDegrees(180));

  public static final Pose2d GRID_2_NODE_1 =
      new Pose2d(
          1.38 + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 + 0.03,
          2.19,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_2_NODE_2 =
      new Pose2d(
          1.38 + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 + 0.03,
          2.75,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_2_NODE_3 =
      new Pose2d(
          1.38 + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 + 0.03,
          3.31,
          Rotation2d.fromDegrees(180));

  public static final Pose2d GRID_3_NODE_1 =
      new Pose2d(
          1.38 + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 + 0.03,
          3.87,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_3_NODE_2 =
      new Pose2d(
          1.38 + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 + 0.03,
          4.42,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_3_NODE_3 =
      new Pose2d(
          1.38 + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 + 0.03,
          4.98,
          Rotation2d.fromDegrees(180));

  public static final Pose2d SINGLE_SUBSYSTEM =
      new Pose2d(
          14.15,
          8.02 - RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 - 0.03,
          Rotation2d.fromDegrees(90));
  public static final Pose2d DOUBLE_SUBSYSTEM =
      new Pose2d(
          16.18 - RobotConfig.getInstance().getRobotWidthWithBumpers() / 2 - 0.03,
          6.68,
          Rotation2d.fromDegrees(0));
}
