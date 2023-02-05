package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.Position;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SetPosition extends CommandBase {
  private Elevator elevator;
  private double rotation;
  private double extension;
  private LoggedDashboardChooser<Position> armChooser;

  /**
   * Constructs a new ElevatorPosition command that will set the elevator to the specified position.
   *
   * @param subsystem the elevator subsystem this command will control
   * @return
   */
  public SetPosition(Elevator subsystem, LoggedDashboardChooser<Position> armChooser) {
    elevator = subsystem;
    this.armChooser = armChooser;

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

    Position position = armChooser.get();

    switch (position) {
      case CONE_STORAGE:
      case CUBE_STORAGE:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CONE_INTAKE_FLOOR:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CONE_INTAKE_SHELF:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CONE_INTAKE_CHUTE:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CONE_HYBRID_LEVEL:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CONE_MID_LEVEL:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CONE_HIGH_LEVEL:
        this.extension = .85;
        this.rotation = 0.7853982;
        break;

      case CUBE_INTAKE_BUMPER:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CUBE_INTAKE_SHELF:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CUBE_INTAKE_CHUTE:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CUBE_HYBRID_LEVEL:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CUBE_MID_LEVEL:
        this.extension = 0;
        this.rotation = 0;
        break;
      case CUBE_HIGH_LEVEL:
        this.extension = 0;
        this.rotation = 0;
        break;
    }

    if (position == Position.CONE_INTAKE_FLOOR) {
      this.extension = 0;
      this.rotation = 0;
    }

    elevator.setPosition(this.rotation, this.extension);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the elevator.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * elevator has reached the specified setpoint, which is slightly above the mid rung.
   */
  @Override
  public boolean isFinished() {
    return elevator.atSetpoint();
  }
}
