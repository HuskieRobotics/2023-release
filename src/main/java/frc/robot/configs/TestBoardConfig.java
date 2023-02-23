package frc.robot.configs;

import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.SwerveModuleConstants.SwerveType;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class TestBoardConfig extends RobotConfig {

  @Override
  public SwerveType getSwerveType() {
    return SwerveType.MK4I;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {0, 0, 0, 0};
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {0, 0, 0, 0};
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {0, 0, 0, 0};
  }

  @Override
  public double[] getSwerveSteerOffsets() {
    return new double[] {0, 0, 0, 0};
  }

  @Override
  public int getGyroCANID() {
    return 0;
  }

  @Override
  public double getTrackwidth() {
    return 0;
  }

  @Override
  public double getWheelbase() {
    return 0;
  }

  @Override
  public double getRobotMaxVelocity() {
    return 0;
  }

  @Override
  public double getAutoMaxSpeed() {
    return 0;
  }

  @Override
  public double getAutoMaxAcceleration() {
    return 0;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return 0;
  }

  @Override
  public int getDriverCameraPort() {
    return 0;
  }
}
