package frc.robot.commands;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.operator_interface.OperatorInterface.GridRow;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.Position;
import frc.robot.subsystems.leds.LEDConstants.RobotStateColors;
import frc.robot.subsystems.leds.LEDs;
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

  private final TunableNumber maxExtensionVelocity =
      new TunableNumber("Elevator/MaxExtensionVel(mps)", MAX_EXTENSION_VELOCITY_METERS_PER_SECOND);
  private final TunableNumber maxExtensionAcceleration =
      new TunableNumber(
          "Elevator/MaxExtensionAccel(mpsps)", EXTENSION_ACCELERATION_METERS_PER_SECOND_PER_SECOND);
  private final TunableNumber maxRetractionVelocity =
      new TunableNumber(
          "Elevator/MaxRetractionVel(mps)", MAX_RETRACTION_VELOCITY_METERS_PER_SECOND);
  private final TunableNumber maxRetractionAcceleration =
      new TunableNumber(
          "Elevator/MaxRetractionAccel(mpsps)",
          RETRACTION_ACCELERATION_METERS_PER_SECOND_PER_SECOND);

  private final TunableNumber fastRotationVelocity =
      new TunableNumber("Elevator/FastRotationVel(dps)", FAST_ROTATION_VELOCITY_DEGREES_PER_SECOND);
  private final TunableNumber meidumRotationVelocity =
      new TunableNumber(
          "Elevator/MediumRotationVel(dps)", MEDIUM_ROTATION_VELOCITY_DEGREES_PER_SECOND);
  private final TunableNumber slowRotationVelocity =
      new TunableNumber("Elevator/SlowRotationVel(dps)", SLOW_ROTATION_VELOCITY_DEGREES_PER_SECOND);

  private final TunableNumber fastRotationAcceleration =
      new TunableNumber(
          "Elevator/FastRotationAccel(dpsps)",
          FAST_ROTATION_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND);
  private final TunableNumber mediumRotationAcceleration =
      new TunableNumber(
          "Elevator/MediumRotationAccel(dpsps)",
          MEDIUM_ROTATION_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND);
  private final TunableNumber slowRotationAcceleration =
      new TunableNumber(
          "Elevator/SlowRotationAccel(dpsps)",
          SLOW_ROTATION_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND);

  // positive values rotation finishes after the extension
  // negative values: extension finishes after the rotation
  private final TunableNumber extensionRotationProfileOffsetIn =
      new TunableNumber("Elevator/ProfileDeltaIn(s)", ROTATION_EXTENSION_TIME_OFFSET_IN);
  private final TunableNumber extensionRotationProfileOffsetOut =
      new TunableNumber("Elevator/ProfileDeltaIn(s)", ROTATION_EXTENSION_TIME_OFFSET_OUT);

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
  public SetElevatorPosition(
      Elevator subsystem, LoggedDashboardChooser<Position> armChooser, LEDs led) {
    this.elevator = subsystem;
    this.armChooser = armChooser;
    this.positionSupplier = () -> Position.INVALID;
    this.led = led;

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

    double extensionCruiseVelocity = maxExtensionVelocity.get();
    double extensionAcceleration = maxExtensionAcceleration.get();
    double rotationCruiseVelocity;
    double rotationAcceleration;

    // positive values rotation starts/finishes after the extension
    // negative values: extension starts/finishes after the rotation
    double rotationExtensionTimeOffset;
    boolean applyTimeOffsetAtStart;

    switch (position) { // extension is in meters, rotation is in radians
      case CONE_STORAGE:
      case CUBE_STORAGE:
        this.extension = CUBE_STORAGE_EXTENSION_POSITION;
        extensionCruiseVelocity = maxRetractionVelocity.get();
        extensionAcceleration = maxRetractionAcceleration.get();
        this.rotation = CUBE_STORAGE_ROTATION_POSITION;
        rotationCruiseVelocity = fastRotationVelocity.get();
        rotationAcceleration = fastRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetIn.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_IN;
        break;
      case AUTO_STORAGE:
        this.extension = AUTO_STORAGE_EXTENSION;
        extensionCruiseVelocity = maxRetractionVelocity.get();
        extensionAcceleration = maxRetractionAcceleration.get();
        this.rotation = AUTO_STORAGE_ROTATION;
        rotationCruiseVelocity = fastRotationVelocity.get();
        rotationAcceleration = fastRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetIn.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_IN;
        this.finishImmediately = true;
        break;
      case CONE_INTAKE_FLOOR:
        this.extension = CONE_GROUND_INTAKE_EXTENSION_POSITION;
        this.rotation = CONE_GROUND_INTAKE_ROTATION_POSITION;
        rotationCruiseVelocity = meidumRotationVelocity.get();
        rotationAcceleration = mediumRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;
      case CONE_INTAKE_SHELF:
      case CUBE_INTAKE_SHELF:
        this.extension = SHELF_EXTENSION_POSITION;
        this.rotation = SHELF_ROTATION_POSITION;
        rotationCruiseVelocity = fastRotationVelocity.get();
        rotationAcceleration = fastRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;
      case CONE_INTAKE_CHUTE:
      case CUBE_INTAKE_CHUTE:
        this.extension = CUBE_CHUTE_EXTENSION_POSITION;
        this.rotation = CUBE_CHUTE_ROTATION_POSITION;
        rotationCruiseVelocity = fastRotationVelocity.get();
        rotationAcceleration = fastRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;
      case CONE_HYBRID_LEVEL:
        this.extension = CONE_HYBRID_EXTENSION_POSITION;
        this.rotation = CONE_HYBRID_ROTATION_POSITION;
        rotationCruiseVelocity = fastRotationVelocity.get();
        rotationAcceleration = fastRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;
      case CONE_MID_LEVEL:
        this.extension = CONE_MID_EXTENSION_POSITION;
        this.rotation = CONE_MID_ROTATION_POSITION;
        rotationCruiseVelocity = fastRotationVelocity.get();
        rotationAcceleration = fastRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;
      case CONE_HIGH_LEVEL:
        this.extension = CONE_HIGH_EXTENSION_POSITION;
        this.rotation = CONE_HIGH_ROTATION_POSITION;
        rotationCruiseVelocity = slowRotationVelocity.get();
        rotationAcceleration = slowRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;
      case CUBE_INTAKE_BUMPER:
        this.extension = CUBE_HYBRID_EXTENSION_POSITION;
        this.rotation = CUBE_HYBRID_ROTATION_POSITION;
        rotationCruiseVelocity = meidumRotationVelocity.get();
        rotationAcceleration = mediumRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;
      case CUBE_HYBRID_LEVEL:
        this.extension = CUBE_HYBRID_EXTENSION_POSITION;
        this.rotation = CUBE_HYBRID_ROTATION_POSITION;
        rotationCruiseVelocity = fastRotationVelocity.get();
        rotationAcceleration = fastRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;
      case CUBE_MID_LEVEL:
        this.extension = CUBE_MID_EXTENSION_POSITION;
        this.rotation = CUBE_MID_ROTATION_POSITION;
        rotationCruiseVelocity = fastRotationVelocity.get();
        rotationAcceleration = fastRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;
      case CUBE_HIGH_LEVEL:
        this.extension = CUBE_HIGH_EXTENSION_POSITION;
        this.rotation = CUBE_HIGH_ROTATION_POSITION;
        rotationCruiseVelocity = meidumRotationVelocity.get();
        rotationAcceleration = mediumRotationAcceleration.get();
        rotationExtensionTimeOffset = extensionRotationProfileOffsetOut.get();
        applyTimeOffsetAtStart = APPLY_TIME_OFFSET_AT_START_OUT;
        break;

      default:
        this.extension = this.elevator.getExtensionElevatorEncoderHeight();
        this.rotation = this.elevator.getRotationElevatorEncoderAngle();
        rotationCruiseVelocity = slowRotationVelocity.get();
        rotationAcceleration = slowRotationAcceleration.get();
        rotationExtensionTimeOffset = 0.0;
        applyTimeOffsetAtStart = false;
        break;
    }

    elevator.initializePosition(this.rotation, this.extension);
    elevator.setPosition(
        this.rotation,
        rotationCruiseVelocity,
        rotationAcceleration,
        this.extension,
        extensionCruiseVelocity,
        extensionAcceleration,
        rotationExtensionTimeOffset,
        applyTimeOffsetAtStart);
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
    return this.finishImmediately || elevator.isAtSetpoint();
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
