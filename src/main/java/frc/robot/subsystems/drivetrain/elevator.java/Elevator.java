package frc.robot.subsystems.drivetrain.Elevator.java;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs(SUBSYSTEM_NAME, inputs);

    double pitch = inputs.picth;
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

  private double getElevatorEncoderHeight() {
    return inputs.rightPosition;
  }
  /** CHECK THIS - not sure about about encoder */
  private double getSetPoint() {
    return encoderPositionSetpoint;
  }
}
