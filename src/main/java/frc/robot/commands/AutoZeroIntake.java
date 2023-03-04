package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * This command, when executed, instructs the intake subsystem to rotate the intake to back to zero.
 *
 * <p>Requires: the Intake subsystem
 *
 * <p>Finished When: The rotation motor begins to stall
 *
 * <p>At End: Sets the position to default position, 0
 */
public class AutoZeroIntake extends CommandBase {
  private Intake intake;

  /**
   * Constructs an AutoZero command that will fully retract the motor.
   *
   * @param subsystem the intake subsystem this command will control
   * @return
   */
  public AutoZeroIntake(Intake subsystem) {
    intake = subsystem;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setRotationMotorPercentage(
        IntakeConstants
            .INTAKE_ROTATION_RETRACTION_POWER); // to change, this is the negative percentage that
    // will retract it
  }

  /**
   * This method will be invoked when this command finishes or is interrupted.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    intake.resetRotationEncoder(IntakeConstants.INTAKE_ROTATION_DEFAULT_POSITION);
    intake.stopIntake();
  }

  /** This method is invoked at the end of each Command Scheduler iteration. */
  @Override
  public boolean isFinished() {
    return intake.isRotationMotorPastCurrentLimit();
  }
}
