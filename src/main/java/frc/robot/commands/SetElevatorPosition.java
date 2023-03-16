package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operator_interface.OperatorInterface.GridRow;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.Position;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.RobotStateColors;
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
  private LEDs led;

  public SetElevatorPosition(Elevator subsystem, Position targetPosition, LEDs led) {
    this(subsystem, () -> targetPosition, led);
  }

  public SetElevatorPosition(
      Elevator subsystem, Supplier<Position> targetPositionSupplier, LEDs led) {
    this.elevator = subsystem;
    this.armChooser = null;
    this.positionSupplier = targetPositionSupplier;
    this.led = led;

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

    led.changeTopStateColor(RobotStateColors.BLINKPINK);

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
        this.extension = ElevatorConstants.CUBE_STORAGE_EXTENSION_POSITION;
        this.rotation = ElevatorConstants.CUBE_STORAGE_ROTATION_POSITION;
        break;
      case AUTO_STORAGE:
        this.extension = ElevatorConstants.AUTO_STORAGE_EXTENSION;
        this.rotation = ElevatorConstants.AUTO_STORAGE_ROTATION;
        this.finishImmediately = true;
        break;
      case CONE_INTAKE_FLOOR:
        this.extension = ElevatorConstants.CONE_GROUND_INTAKE_ROTATION_POSITION;
        this.rotation = ElevatorConstants.CONE_GROUND_INTAKE_EXTENSION_POSITION;
        break;
      case CONE_INTAKE_SHELF:
      case CUBE_INTAKE_SHELF:
        this.extension = ElevatorConstants.SHELF_EXTENSION_POSITION;
        this.rotation = ElevatorConstants.SHELF_ROTATION_POSITION;
        break;
      case CONE_INTAKE_CHUTE:
      case CUBE_INTAKE_CHUTE:
        this.extension = ElevatorConstants.CUBE_CHUTE_EXTENSION_POSITION;
        this.rotation = ElevatorConstants.CUBE_CHUTE_ROTATION_POSITION;
        break;
      case CONE_HYBRID_LEVEL:
        this.extension = ElevatorConstants.CONE_HYBRID_EXTENSION_POSITION;
        this.rotation = ElevatorConstants.CONE_HYBRID_ROTATION_POSITION;
        break;
      case CONE_MID_LEVEL:
        this.extension = ElevatorConstants.CONE_MID_EXTENSION_POSITION;
        this.rotation = ElevatorConstants.CONE_MID_ROTATION_POSITION; // 48.0
        break;
      case CONE_HIGH_LEVEL:
        this.extension = ElevatorConstants.CONE_HIGH_EXTENSION_POSITION;
        this.rotation = ElevatorConstants.CONE_HIGH_EXTENSION_POSITION;
        break;
      case CUBE_INTAKE_BUMPER:
        this.extension = ElevatorConstants.CONE_HYBRID_ROTATION_POSITION;
        this.rotation = ElevatorConstants.CUBE_HYBRID_ROTATION_POSITION;
        break;
      case CUBE_HYBRID_LEVEL:
        this.extension = ElevatorConstants.CUBE_HYBRID_EXTENSION_POSITION;
        this.rotation = ElevatorConstants.CUBE_HYBRID_ROTATION_POSITION;
        break;
      case CUBE_MID_LEVEL:
        this.extension = ElevatorConstants.CUBE_MID_EXTENSION_POSITION;
        this.rotation = ElevatorConstants.CUBE_MID_ROTATION_POSITION;
        break;
      case CUBE_HIGH_LEVEL:
        this.extension = ElevatorConstants.CUBE_HIGH_EXTENSION_POSITION;
        this.rotation = ElevatorConstants.CUBE_HIGH_ROTATION_POSITION;
        break;
    }

    elevator.initializePosition(this.rotation, this.extension);
  }

  @Override
  public void execute() {
    // FIXME: need to query the intake subsystem to determine the position of the intake, replace
    // true with isIntakeEnabled()
    // led.changeTopStateColor(RobotStateColors.BLINKPINK);
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
    led.changeTopStateColor(RobotStateColors.WHITE);
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
