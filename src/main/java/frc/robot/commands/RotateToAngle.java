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
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RotateToAngle extends CommandBase {
  protected final Drivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  protected Pose2d targetPose;

  protected boolean running = false;

  protected static final TunableNumber thetaKp = new TunableNumber("RotateToAngle/ThetaKp", 2);
  protected static final TunableNumber thetaKd = new TunableNumber("RotateToAngle/ThetaKd", 0.1);
  protected static final TunableNumber thetaKi =
      new TunableNumber("RotateToAngle/ThetaKi", RobotConfig.getInstance().getDriveToPoseThetaKI());
  protected static final TunableNumber thetaMaxVelocity =
      new TunableNumber("RotateToAngle/ThetaMaxVelocity", 8);
  protected static final TunableNumber thetaMaxAcceleration =
      new TunableNumber("RotateToAngle/ThetaMaxAcceleration", 100);
  protected static final TunableNumber thetaTolerance =
      new TunableNumber(
          "RotateToAngle/ThetaTolerance", RobotConfig.getInstance().getDriveToPoseThetaTolerance());

  protected final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          thetaKi.get(),
          thetaKd.get(),
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
          LOOP_PERIOD_SECS);

  /** Drives to the specified pose under full software control. */
  public RotateToAngle(Drivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/RotateToAngle", true);

    // Reset all controllers
    Pose2d currentPose = drivetrain.getPose();
    thetaController.reset(currentPose.getRotation().getRadians());
    thetaController.setTolerance(thetaTolerance.get());
    this.targetPose = poseSupplier.get();

    Logger.getInstance().recordOutput("RotateToAngle/targetPose", targetPose);
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (thetaKp.hasChanged()
        || thetaKd.hasChanged()
        || thetaKi.hasChanged()
        || thetaMaxVelocity.hasChanged()
        || thetaMaxAcceleration.hasChanged()) {
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
      thetaController.setI(thetaKi.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    // Get current and target pose
    Pose2d currentPose = drivetrain.getPose();

    // Command speeds
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    drivetrain.drive(0.0, 0.0, thetaVelocity, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    running = false;
    Logger.getInstance().recordOutput("ActiveCommands/RotateToAngle", false);
  }

  @Override
  public boolean isFinished() {
    Logger.getInstance().recordOutput("RotateToAngle/tErr", thetaController.atGoal());

    return (running && thetaController.atGoal());
  }
}
