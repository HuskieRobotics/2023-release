package frc.robot.subsystems.drivetrain.elevator.java;

public class ElevatorIOHardware implements ElevatorIO {
  private boolean isControlEnabled;

  /*
   * Make sure to add the motors and Pigeon later
   *  EX: private final WPI_TalonFX leftElevatorMoter
   */

  public ElevatorIOTalonFX() {
    CANDeviceFinder can = new CANDeviceFinder();

    this.isControlEnabled = false;
  }
}
