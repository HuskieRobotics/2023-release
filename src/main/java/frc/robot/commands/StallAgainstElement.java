// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Field2d;
import frc.robot.FieldRegionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class StallAgainstElement extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose;
  private Timer timer;
  private boolean isEndNode;
  private double timeout;

  // FIXME: tune these values
  private static final double SQUARING_CURRENT_AMPS = 45.0;

  private static final TunableNumber squaringSpeed =
      new TunableNumber(
          "StallAgainstElement/SquaringSpeed", RobotConfig.getInstance().getSquaringSpeed());
  private static final TunableNumber squaringCurrent =
      new TunableNumber("StallAgainstElement/SquaringCurrent", SQUARING_CURRENT_AMPS);

  /** Drives to the specified pose under full software control. */
  public StallAgainstElement(Drivetrain drivetrain, Supplier<Pose2d> poseSupplier, double timeout) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.timeout = timeout;
    this.timer = new Timer();
    this.isEndNode = false;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/StallAgainstElement", true);
    this.targetPose = poseSupplier.get();
    if (targetPose.equals(
            Field2d.getInstance()
                .mapPoseToCurrentAlliance(
                    new Pose2d(
                        FieldRegionConstants.GRID_1_NODE_1.getX()
                            + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2
                            + Units.inchesToMeters(2),
                        FieldRegionConstants.GRID_1_NODE_1.getY(),
                        FieldRegionConstants.GRID_1_NODE_1.getRotation())))
        || targetPose.equals(
            Field2d.getInstance()
                .mapPoseToCurrentAlliance(
                    new Pose2d(
                        FieldRegionConstants.GRID_3_NODE_3.getX()
                            + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2
                            + Units.inchesToMeters(2),
                        FieldRegionConstants.GRID_3_NODE_3.getY(),
                        FieldRegionConstants.GRID_3_NODE_3.getRotation())))) {
      isEndNode = true;
    }
    this.timer.restart();
  }

  @Override
  public void execute() {
    Rotation2d rotation = this.targetPose.getRotation();
    double xVelocity = squaringSpeed.get() * rotation.getCos();
    double yVelocity = squaringSpeed.get() * rotation.getSin();

    drivetrain.drive(xVelocity, yVelocity, 0.0, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    this.isEndNode = false;
    Logger.getInstance().recordOutput("ActiveCommands/StallAgainstElement", false);
  }

  @Override
  public boolean isFinished() {
    return !drivetrain.isMoveToGridEnabled()
        || isEndNode
        || this.timer.hasElapsed(this.timeout)
        || (drivetrain.getAverageDriveCurrent() > squaringCurrent.get());
  }
}
