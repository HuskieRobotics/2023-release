package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.Position;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
public class SetElevatorPositionBeforeRetraction extends SetElevatorPosition {

  private static final TunableNumber additionalExtension =
      new TunableNumber("SetElevatorPositionBeforeRetraction/additionalExtension", 5);

  public SetElevatorPositionBeforeRetraction(Elevator subsystem, Position targetPosition) {
    super(subsystem, targetPosition);
  }

  public SetElevatorPositionBeforeRetraction(
      Elevator subsystem, Supplier<Position> targetPositionSupplier) {
    super(subsystem, targetPositionSupplier);
  }

  /**
   * This method is invoked once when this command is scheduled. It sets the setpoint of the
   * elevator position to slightly above the mid rung. It is critical that this initialization
   * occurs in this method and not the constructor as this command is constructed once when the
   * RobotContainer is created, but this method is invoked each time this command is scheduled.
   */
  @Override
  public void initialize() {

    Logger.getInstance().recordOutput("ActiveCommands/SetElevatorPositionBeforeRetraction", true);

    super.initialize();

    this.rotation = this.rotation + Units.degreesToRadians(additionalExtension.get());

    elevator.initializePosition(this.rotation, this.extension);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    Logger.getInstance().recordOutput("ActiveCommands/SetElevatorPositionBeforeRetraction", false);
  }
}
