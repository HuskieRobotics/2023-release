package main.java.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * This command, when executed, extends the climber slightly above the mid rung in preparation to
 * climb the mid rung.
 *
 * <p>Requires: the Elevator subsystem
 *
 * <p>Finished When: the climber is positioned slightly above the mid rung
 *
 * <p>At End: stops the elevator
 */
public class SetElevatorExtension extends CommandBase {
  private final Elevator elevator;

  /**
   * Constructs a new ExtendClimberToMidRungCommand object.
   *
   * @param subsystem the elevator subsystem this command will control
   */
  public SetElevatorExtension(Elevator subsystem, double rotation) {
    elevator = subsystem;
    addRequirements(elevator);
  }

  /**
   * This method is invoked once when this command is scheduled. It sets the setpoint of the
   * elevator position to slightly above the mid rung. It is critical that this initialization
   * occurs in this method and not the constructor as this command is constructed once when the
   * RobotContainer is created, but this method is invoked each time this command is scheduled.
   */
  @Override
  public void initialize() {
  
  }

  @Override
  public void execute() {
    elevator.setRotationMotorEncoderAngle(rotation);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the elevator.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    elevator.stopRotation();
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * elevator has reached the specified setpoint, which is slightly above the mid rung.
   */
  @Override
  public boolean isFinished() {
    return elevator.atRotationSetPoint();
  }
}