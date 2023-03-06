package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operator_interface.OperatorInterface.GridRow;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.Position;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This command, when executed, instructs the elevator subsystem to extend and rotate the elevator
 * to the specified positions.
 *
 * <p>Requires: the Elevator subsystem
 *
 * <p>Finished When: the elevator is at the specified positions (within the defined tolerance)
 *
 * <p>At End: continues to hold the elevator at the specified positions
 */
public class SetElevatorPosition extends CommandBase {
  protected Elevator elevator;
  protected double rotation;
  protected double extension;
  private Supplier<Position> positionSupplier;
  private LoggedDashboardChooser<Position> armChooser;
  private boolean finishImmediately;

  public SetElevatorPosition(Elevator subsystem, Position targetPosition) {
    this(subsystem, () -> targetPosition);
  }

  public SetElevatorPosition(Elevator subsystem, Supplier<Position> targetPositionSupplier) {
    this.elevator = subsystem;
    this.armChooser = null;
    this.positionSupplier = targetPositionSupplier;

    addRequirements(elevator);
  }

  /**
   * Constructs a new ElevatorPosition command that will set the elevator to the specified position.
   *
   * @param subsystem the elevator subsystem this command will control
   * @return
   */
  public SetElevatorPosition(Elevator subsystem, LoggedDashboardChooser<Position> armChooser) {
    this.elevator = subsystem;
    this.armChooser = armChooser;
    this.positionSupplier = () -> Position.INVALID;

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

    Logger.getInstance().recordOutput("ActiveCommands/SetElevatorPosition", true);

    this.finishImmediately = false;

    Position position = positionSupplier.get();

    if (armChooser != null) {
      position = armChooser.get();
    }

    // check if we are toggled for cones or cubes; if cubes, adjust the position
    switch (position) {
      case CONE_HYBRID_LEVEL:
        if (!this.elevator.getToggledToCone()) {
          position = Position.CUBE_HYBRID_LEVEL;
        }
        break;
      case CONE_MID_LEVEL:
        if (!this.elevator.getToggledToCone()) {
          position = Position.CUBE_MID_LEVEL;
        }
        break;
      case CONE_HIGH_LEVEL:
        if (!this.elevator.getToggledToCone()) {
          position = Position.CUBE_HIGH_LEVEL;
        }
        break;
      default:
        break;
    }

    switch (position) { // extension is in meters, rotation is in radians
      case INVALID:
        this.extension = this.elevator.getExtensionElevatorEncoderHeight();
        this.rotation = this.elevator.getRotationElevatorEncoderAngle();
        break;
      case CONE_STORAGE:
      case CUBE_STORAGE:
        this.extension = 0.0;
        this.rotation = Units.degreesToRadians(90.0 - 24.173);
        break;
      case AUTO_STORAGE:
        this.extension = Units.inchesToMeters(34);
        this.rotation = Units.degreesToRadians(90.0 - 24.173);
        this.finishImmediately = true;
        break;
      case CONE_INTAKE_FLOOR:
        this.extension = Units.inchesToMeters(34);
        this.rotation = Units.degreesToRadians(90.0 - 77.0);
        break;
      case CONE_INTAKE_SHELF:
      case CUBE_INTAKE_SHELF:
        this.extension = Units.inchesToMeters(45);
        this.rotation = Units.degreesToRadians(90.0 - 42.0);
        break;
      case CONE_INTAKE_CHUTE:
      case CUBE_INTAKE_CHUTE:
        this.extension = Units.inchesToMeters(24);
        this.rotation = Units.degreesToRadians(90.0 - 27.0);
        break;
      case CONE_HYBRID_LEVEL:
        this.extension = Units.inchesToMeters(19);
        this.rotation = Units.degreesToRadians(90.0 - 60.0);
        break;
      case CONE_MID_LEVEL:
        this.extension = Units.inchesToMeters(44);
        this.rotation = Units.degreesToRadians(90.0 - 44.0); // 48.0
        break;
      case CONE_HIGH_LEVEL:
        this.extension = Units.inchesToMeters(65);
        this.rotation = Units.degreesToRadians(90.0 - 44.0);
        break;
      case CUBE_INTAKE_BUMPER:
        this.extension = Units.inchesToMeters(8);
        this.rotation = Units.degreesToRadians(90.0 - 43.0);
        break;
      case CUBE_HYBRID_LEVEL:
        this.extension = Units.inchesToMeters(20.98);
        this.rotation = Units.degreesToRadians(90.0 - 53.88);
        break;
      case CUBE_MID_LEVEL:
        this.extension = Units.inchesToMeters(40.63);
        this.rotation = Units.degreesToRadians(90.0 - 51.55);
        break;
      case CUBE_HIGH_LEVEL:
        this.extension = Units.inchesToMeters(58);
        this.rotation = Units.degreesToRadians(90.0 - 53.0);
        break;
    }

    elevator.initializePosition(this.rotation, this.extension);
  }

  @Override
  public void execute() {
    // FIXME: need to query the intake subsystem to determine the position of the intake, replace
    // true with isIntakeEnabled()
    elevator.setPosition(this.rotation, this.extension, true);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the elevator.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().recordOutput("ActiveCommands/SetElevatorPosition", false);
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * elevator has reached the specified setpoint, which is slightly above the mid rung.
   */
  @Override
  public boolean isFinished() {
    return this.finishImmediately || elevator.atSetpoint();
  }

  public static Position convertGridRowToPosition(GridRow row) {
    switch (row) {
      case BOTTOM:
        return Position.CONE_HYBRID_LEVEL;
      case MIDDLE:
        return Position.CONE_MID_LEVEL;
      case TOP:
        return Position.CONE_HIGH_LEVEL;
      default:
        return Position.CONE_HYBRID_LEVEL;
    }
  }
}
