package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.team6328.util.FieldConstants;
import java.awt.geom.*;
import java.util.HashMap;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Region2d {
  private Path2D shape;
  private HashMap<Region2d, Translation2d> transitionMap;
  private DriverStation.Alliance alliance = DriverStation.Alliance.Invalid;
  /**
   * Create a Region2d, a polygon, from an array of Translation2d specifying vertices of a polygon.
   * The polygon is created using the even-odd winding rule.
   *
   * @param points the array of Translation2d that define the vertices of the region.
   */
  public Region2d(Translation2d[] points) {
    this.transitionMap = new HashMap<>();
    this.shape = new Path2D.Double(Path2D.WIND_EVEN_ODD, points.length);
    this.shape.moveTo(points[0].getX(), points[0].getY());

    for (int i = 1; i < points.length; i++) {
      this.shape.lineTo(points[i].getX(), points[i].getY());
    }

    this.shape.closePath();
  }

  public void logPoints() {

    // assume all regions are rectangular
    Rectangle2D rect = this.shape.getBounds2D();
    Logger.getInstance()
        .recordOutput("Region2d/point0", new Pose2d(rect.getX(), rect.getY(), new Rotation2d()));
    Logger.getInstance()
        .recordOutput(
            "Region2d/point1",
            new Pose2d(rect.getX() + rect.getWidth(), rect.getY(), new Rotation2d()));
    Logger.getInstance()
        .recordOutput(
            "Region2d/point2",
            new Pose2d(rect.getX(), rect.getY() + rect.getHeight(), new Rotation2d()));
    Logger.getInstance()
        .recordOutput(
            "Region2d/point3",
            new Pose2d(
                rect.getX() + rect.getWidth(), rect.getY() + rect.getHeight(), new Rotation2d()));

    for (int i = 0; i < 4; i++) {
      Logger.getInstance().recordOutput("Region2d/transition" + i, new Pose2d());
    }
    int i = 0;
    for (Region2d region : transitionMap.keySet()) {
      Translation2d point = transitionMap.get(region);
      Logger.getInstance()
          .recordOutput(
              "Region2d/transition" + i, new Pose2d(point.getX(), point.getY(), new Rotation2d()));
      i++;
    }
  }

  /**
   * Returns true if the region contains a given Pose2d
   *
   * @param other the given pose2d
   * @return if the pose is inside the region
   */
  public boolean contains(Pose2d other) {

    if (this.alliance == DriverStation.Alliance.Red) {
      return this.shape.contains(
          new Point2D.Double(other.getX(), FieldConstants.fieldWidth - other.getY()));
    } else {
      return this.shape.contains(new Point2D.Double(other.getX(), other.getY()));
    }
  }

  /**
   * Add a neighboring Region2d and the ideal point through which to transition between the two.
   * Normally, this operation will be performed once per transition with the transition point on at
   * the center of the boundary between the two regions.
   *
   * @param other the other region
   * @param point a Translation2d representing the transition point
   */
  public void addNeighbor(Region2d other, Translation2d point) {
    transitionMap.put(other, point);
  }

  /**
   * Returns a Set of this region's neighbors
   *
   * @return
   */
  public Set<Region2d> getNeighbors() {
    return transitionMap.keySet();
  }

  /**
   * Get the transition pont between this region and another region. Returns null if they aren't a
   * neighbor.
   *
   * @param other the other region
   * @return the transition point, represented as a Translation2d
   */
  public Translation2d getTransitionPoint(Region2d other) {
    if (getNeighbors().contains(other)) {

      if (this.alliance == DriverStation.Alliance.Red) {
        return new Translation2d(
            transitionMap.get(other).getX(),
            FieldConstants.fieldWidth - transitionMap.get(other).getY());
      } else {
        return transitionMap.get(other);
      }

    } else {
      return null;
    }
  }

  public void updateAlliance(DriverStation.Alliance newAlliance) {
    this.alliance = newAlliance;
  }
}
