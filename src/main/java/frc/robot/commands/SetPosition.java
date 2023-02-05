package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.Position;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SetPosition extends CommandBase {
  private Elevator elevator;
  private double rotation;
  private double extension;

  private final LoggedDashboardChooser<Position> armChooser =
      new LoggedDashboardChooser<>("Arm Position");
  /**
   * Constructs a new ElevatorPosition command that will set the elevator to the specified position.
   *
   * @param subsystem the elevator subsystem this command will control
   * @return
   */
  public SetPosition(Elevator subsystem) {
    elevator = subsystem;

    armChooser.addDefaultOption("CONE_STORAGE", Position.CONE_STORAGE);
    armChooser.addOption("CUBE_STORAGE", Position.CUBE_STORAGE);
    armChooser.addOption("CONE_INTAKE_FLOOR", Position.CONE_INTAKE_FLOOR);
    armChooser.addOption("CUBE_INTAKE_BUMPER", Position.CUBE_INTAKE_BUMPER);
    armChooser.addOption("CONE_INTAKE_SHELF", Position.CONE_INTAKE_SHELF);
    armChooser.addOption("CUBE_INTAKE_SHELF", Position.CUBE_INTAKE_SHELF);
    armChooser.addOption("CONE_INTAKE_CHUTE", Position.CONE_INTAKE_CHUTE);
    armChooser.addOption("CUBE_INTAKE_CHUTE", Position.CUBE_INTAKE_CHUTE);
    armChooser.addOption("CONE_HYBRID_LEVEL", Position.CONE_HYBRID_LEVEL);
    armChooser.addOption("CONE_MID_LEVEL", Position.CONE_MID_LEVEL);
    armChooser.addOption("CONE_HIGH_LEVEL", Position.CONE_HIGH_LEVEL);
    armChooser.addOption("CUBE_HYBRID_LEVEL", Position.CUBE_HYBRID_LEVEL);
    armChooser.addOption("CUBE_MID_LEVEL", Position.CUBE_MID_LEVEL);
    armChooser.addOption("CUBE_HIGH_LEVEL", Position.CUBE_HIGH_LEVEL);

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
        this.extension = 0;
        this.rotation = 0;
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
