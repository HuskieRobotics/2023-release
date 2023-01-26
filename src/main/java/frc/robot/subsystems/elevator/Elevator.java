package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
// import frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {

  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private final ElevatorIOSim sim = new ElevatorIOSim();
  public double rotationSetpoint = 0.0;
  public double extensionSetpoint = 0.0;

  private Encoder m_Encoder = sim.getElevatorEncoder();
  public ElevatorIO io;

  public Elevator(ElevatorIO io) {
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
  public void setElevatorExtension(double desiredEncoderPosition) {
    setExtensionMotorPosition(desiredEncoderPosition, ARBITRARY_FEED_FORWARD_EXTEND);
    this.extensionSetpoint = desiredEncoderPosition;
  }

  /**
   * @param desiredEncoderPosition this may not be an encoder, check in teh future
   */
  public void setElevatorRotation(double desiredEncoderPosition) {
    setRotationMotorPosition(desiredEncoderPosition, ARBITRARY_FEED_FORWARD_EXTEND);
    this.rotationSetpoint = desiredEncoderPosition;
  }

  public boolean atExtensionSetpoint() {
    return inputs.extensionPosition == extensionSetpoint; //FIXME add deadzone
  }

  public boolean atRotationSetpoint() {
    return inputs.rotationPosition == rotationSetpoint; //FIXME add deadzone
  }

  public void stopExtension() {
    io.setExtensionMotorPercentage(0.0);
  }

  public void stopRotation() {
    io.setRotationMotorPercentage(0.0);
  }
  
  private double getExtensionElevatorEncoderHeight() {
     return inputs.extensionPosition;
  }

  private double getRotationElevatorEncoderHeight() {
    return inputs.rotationPosition;
 }
}
