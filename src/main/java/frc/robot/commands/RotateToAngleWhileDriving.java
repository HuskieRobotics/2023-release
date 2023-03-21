package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RotateToAngleWhileDriving extends RotateToAngle {
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  public RotateToAngleWhileDriving(
      Drivetrain drivetrain,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    super(drivetrain, poseSupplier);
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
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

    drivetrain.drive(
        translationXSupplier.getAsDouble(),
        translationYSupplier.getAsDouble(),
        thetaVelocity,
        true,
        true);
  }
}
