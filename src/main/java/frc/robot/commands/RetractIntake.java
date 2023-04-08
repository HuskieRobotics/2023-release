package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.*;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the intake subsystem to rotate the intake to the specified
 * position and, based on that position, runs the roller at the corresponding speed.
 *
 * <p>Requires: the Intake subsystem
 *
 * <p>Finished When: the intake is at the specified position
 *
 * <p>At End: stop the rotation (relying on the gearbox to hold the position) and leaves the roller
 * running
 */
public class RetractIntake extends CommandBase {
  private Intake intake;

  /**
   * Constructs a new SetIntakeState command that changes based on the given state.
   *
   * @param subsystem the intake subsystem this command will control
   * @return
   */
  public RetractIntake(Intake subsystem) {
    intake = subsystem;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/RetractIntake", true);
    intake.retract();
    intake.stopRoller();
  }

  /**
   * This method will be invoked when this command finishes or is interrupted.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().recordOutput("ActiveCommands/RetractIntake", false);
  }

  /** This method is invoked at the end of each Command Scheduler iteration. */
  @Override
  public boolean isFinished() {
    return intake.isRetracted();
  }
}
