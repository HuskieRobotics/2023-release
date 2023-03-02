// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Field2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose;

  private boolean running = false;

  private static final TunableNumber driveKp =
      new TunableNumber("DriveToPose/DriveKp", RobotConfig.getInstance().getDriveToPoseDriveKP());
  private static final TunableNumber driveKd =
      new TunableNumber("DriveToPose/DriveKd", RobotConfig.getInstance().getDriveToPoseDriveKD());
  private static final TunableNumber thetaKp =
      new TunableNumber("DriveToPose/ThetaKp", RobotConfig.getInstance().getDriveToPoseThetaKP());
  private static final TunableNumber thetaKd =
      new TunableNumber("DriveToPose/ThetaKd", RobotConfig.getInstance().getDriveToPoseThetaKD());
  private static final TunableNumber driveMaxVelocity =
      new TunableNumber(
          "DriveToPose/DriveMaxVelocity",
          RobotConfig.getInstance().getDriveToPoseDriveMaxVelocity());
  private static final TunableNumber driveMaxAcceleration =
      new TunableNumber(
          "DriveToPose/DriveMaxAcceleration",
          RobotConfig.getInstance().getDriveToPoseDriveMaxAcceleration());
  private static final TunableNumber thetaMaxVelocity =
      new TunableNumber(
          "DriveToPose/ThetaMaxVelocity",
          RobotConfig.getInstance().getDriveToPoseTurnMaxVelocity());
  private static final TunableNumber thetaMaxAcceleration =
      new TunableNumber(
          "DriveToPose/ThetaMaxAcceleration",
          RobotConfig.getInstance().getDriveToPoseTurnMaxAcceleration());
  private static final TunableNumber driveTolerance =
      new TunableNumber(
          "DriveToPose/DriveTolerance", RobotConfig.getInstance().getDriveToPoseDriveTolerance());
  private static final TunableNumber thetaTolerance =
      new TunableNumber(
          "DriveToPose/ThetaTolerance", RobotConfig.getInstance().getDriveToPoseThetaTolerance());

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          driveKp.get(),
          0.0,
          driveKd.get(),
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          driveKp.get(),
          0.0,
          driveKd.get(),
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          0.0,
          thetaKd.get(),
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  // ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain").addDouble("X Error",
  // xController::getPositionError);
  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drivetrain drivetrain, Pose2d pose) {
    this(drivetrain, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/DriveToPose", true);

    

    // Reset all controllers
    Pose2d currentPose = drivetrain.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
    xController.setTolerance(driveTolerance.get());
    yController.setTolerance(driveTolerance.get());
    thetaController.setTolerance(thetaTolerance.get());

    this.targetPose = Field2d.getInstance().mapPoseToCurrentAlliance(poseSupplier.get());
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveKp.hasChanged()
        || driveKd.hasChanged()
        || thetaKp.hasChanged()
        || thetaKd.hasChanged()
        || driveMaxVelocity.hasChanged()
        || driveMaxAcceleration.hasChanged()
        || thetaMaxVelocity.hasChanged()
        || thetaMaxAcceleration.hasChanged()) {
      xController.setP(driveKp.get());
      xController.setD(driveKd.get());
      xController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      xController.setTolerance(driveTolerance.get());
      yController.setP(driveKp.get());
      yController.setD(driveKd.get());
      yController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      yController.setTolerance(driveTolerance.get());
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    // Get current and target pose
    Pose2d currentPose = drivetrain.getPose();

    // Command speeds
    double xVelocity = xController.calculate(currentPose.getX(), this.targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), this.targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());
    if (xController.atGoal()) xVelocity = 0.0;
    if (yController.atGoal()) yVelocity = 0.0;
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    drivetrain.drive(xVelocity, yVelocity, thetaVelocity, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    running = false;
    Logger.getInstance().recordOutput("ActiveCommands/DriveToPose", false);
  }

  @Override
  public boolean isFinished() {
    return running && xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
