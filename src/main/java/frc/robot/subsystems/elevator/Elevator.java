package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

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
  Logger.getInstance().processInputs(SUBSYSTEM_NAME, inputs);
  }

  public void setElevatorExtensionMotorPower(double power) {
    if(
      (this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500 && power > 0) ||
      (this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500 && power < 0)) {
      stopExtension();
    }
    else {
      io.setExtensionMotorPercentage(power);
    }
  }
  public void setElevatorRotationMotorPower(double power) {
    if(
      (this.getRotationElevatorEncoderAngle() > MAX_ROTATION_POSITION - 2500 && power > 0) ||
      (this.getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500 && power < 0)) {
      stopExtension();
    }
    else {
      io.setRotationMotorPercentage(power);
    }
  }

  public void setElevatorExtension(double desiredEncoderPosition) {
    if(
      (this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500 && power > 0) ||
      (this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500 && power < 0)) {
      stopExtension();
    }
    else{
      if(getRotationElevatorEncoderAngle() < MIN_ELEVATOR_EXTENSION_ANGLE){
        setRotationMotorPosition(elevator, MIN_ELEVATOR_EXTENSION_ANGLE);
      }
      this.extensionSetpoint = desiredEncoderPosition;
      io.setExtensionnPosition(desiredEncoderPosition, ARBITRARY_FEED_FORWARD_EXTENSION);
    }
  }

  public void setElevatorRotation(double desiredEncoderPosition) {
    if(
      (this.getRotationElevatorEncoderAngle() > MAX_ROTATION_POSITION - 2500 && power > 0) ||
      (this.getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500 && power < 0)) {
      this.stopExtension();
    }
    else {
      io.setRotationPosition(desiredEncoderPosition, ARBITRARY_FEED_FORWARD_ROTATION);
      this.rotationSetpoint = desiredEncoderPosition;
    }
  }

  public boolean atExtensionSetpoint() {
    return Math.abs(this.inputs.extensionPosition - extensionSetpoint) < ELEVATOR_EXTENSION_POSITION_TOLERANCE; 
  }

  public boolean atRotationSetpoint() {
    return Math.abs(this.inputs.rotationPosition - rotationSetpoint) < ELEVATOR_ROTATION_POSITION_TOLERANCE;
  }

  public void stopExtension() {
    this.io.setExtensionMotorPercentage(0.0);
  }

  public void stopRotation() {
    this.io.setRotationMotorPercentage(0.0);
  }

  public void stopElevator() {
    this.stopExtension();
    this.stopRotation();
  }
  
  public double getExtensionElevatorEncoderHeight() {
     return this.inputs.extensionPosition;
  }

  public double getRotationElevatorEncoderAngle() {
    return this.nputs.rotationPosition;
 }

  public void setPosition(double rotation, double extension) {
    this.setElevatorExtension(extension);
    this.setElevatorRotation(rotation);
  }

  public boolean atSetpoint() { 
    return this.atRotationSetpoint() && this.atExtensionSetpoint();
  }
}
