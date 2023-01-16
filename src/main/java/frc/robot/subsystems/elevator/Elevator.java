package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
// import frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final ElevatorIOInputs io;
  private final ElevatorIOHardware inputs = new ElevatorIOHardware();

  public Elevator(ElevatorIOInputs io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.getInstance().processInputs(SUBSYSTEM_NAME, inputs);

    // double pitch = inputs.pitch;
  }

  public void setElevatorMotorPower(double power) {}

  /**
   * @param desiredEncoderPosition this may not be an encoder, check in teh future
   */
  public void setElevatorMotorPosition(double desiredEncoderPosition) {}

  public boolean atSetpoint() {
    return true;
  }

  public void stopElevator() {
    io.setMotorPercentage(0.0);
  }

  public void enableElevatorControl() {
    io.setControlEnabled(true);
  }

  public void disableElevatorControl() {
    io.setControlEnabled(false);
  }

  // private double getElevatorEncoderHeight() {
  //    return inputs.rightPosition;
  // }
  /** CHECK THIS - not sure about about encoder */
  private double getSetPoint() {
    return 10;
  }
}
