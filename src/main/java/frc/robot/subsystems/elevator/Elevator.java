package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private final ElevatorIOSim sim = new ElevatorIOSim();
  public double rotationSetpoint = 0.0;
  public double extensionSetpoint = 0.0;
  public double power = 0.0;

  public boolean isControlEnabled = false;

  private Encoder m_Encoder = sim.getElevatorEncoder();
  public ElevatorIO io;
  public int valueListenerHandle;

  public Elevator(ElevatorIO io) {
    this.io = io;

    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    // get the default instance of NetworkTables
    NetworkTableInstance instListner = NetworkTableInstance.getDefault();

    if (DEBUGGING) {
      tab.add("elevator", this);
      tab.addNumber("Encoder", this::getExtensionElevatorEncoderHeight);
    }

    if (TESTING) {
      tab.add("Set Extension Position", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -1, "max", 1))
          .getEntry(); // FIXME create two sliders for rotation and extension

      tab.add("Set Rotation Position", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -1, "max", 1))
          .getEntry();
    }

    if (TUNING) {
      io.setControlEnabled(true);

      tab.addNumber("Rotation Closed Loop Target", this::getExtensionSetpoint);
      tab.addNumber("Rotation Closed Loop Error", () -> inputs.rotationClosedLoopError);
      tab.addNumber("Rotation Velocity", () -> inputs.rotationVelocity);
      tab.addNumber("Rotation Right Motor Volts", () -> inputs.rotationAppliedVolts);

      tab.addNumber("Extension Closed Loop Target", this::getRotationSetpoint);
      tab.addNumber("Extension Closed Loop Error", () -> inputs.extensionClosedLoopError);
      tab.addNumber("Extension Velocity", () -> inputs.extensionVelocity);
      tab.addNumber("Extension Left Motor Volts", () -> inputs.extensionAppliedVolts); // FIXME
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);
  }

  public void setElevatorExtensionMotorPower(double power) {
    if ((this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500 && power > 0)
        || (this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500
            && power < 0)) {
      stopExtension();
    } else {
      io.setExtensionMotorPercentage(power);
    }
  }

  public void setElevatorRotationMotorPower(double power) {
    if ((this.getRotationElevatorEncoderAngle() > MAX_ROTATION_POSITION - 2500 && power > 0)
        || (this.getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500 && power < 0)) {
      stopRotation();
    } else {
      io.setRotationMotorPercentage(power);
    }
  }

  public void setElevatorExtension(double extension) {
    if ((this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500 && power > 0)
        || (this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500
            && power < 0)) {
      stopExtension();
    } else {
      if (getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500) {
        io.setRotationPosition(MIN_ROTATION_POSITION + 2500, ARBITRARY_FEED_FORWARD_ROTATION);
      }
      this.extensionSetpoint = extension;
      io.setExtensionPosition(extensionSetpoint, ARBITRARY_FEED_FORWARD_EXTENSION);
    }
  }

  public void setElevatorRotation(Double rotation) {
    if ((this.getRotationElevatorEncoderAngle() > MAX_ROTATION_POSITION - 2500 && power > 0)
        || (this.getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500 && power < 0)) {
      this.stopExtension();
    } else {
      io.setRotationPosition(rotation, ARBITRARY_FEED_FORWARD_ROTATION);
      this.rotationSetpoint = rotation;
    }
  }

  public boolean atExtensionSetpoint() {
    return Math.abs(this.inputs.extensionPosition - extensionSetpoint)
        < ELEVATOR_EXTENSION_POSITION_TOLERANCE;
  }

  public boolean atRotationSetpoint() {
    return Math.abs(this.inputs.rotationPosition - rotationSetpoint)
        < ELEVATOR_ROTATION_POSITION_TOLERANCE;
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
    return this.inputs.rotationPosition;
  }

  public void setPosition(Double rotation, Double extension) {
    this.setElevatorExtension(extension);
    this.setElevatorRotation(rotation);
  }

  public boolean atSetpoint() {
    return this.atRotationSetpoint() && this.atExtensionSetpoint();
  }

  public void setControlEnabled() {
    this.isControlEnabled = true;
  }

  public boolean nearExtensionMaximum() {
    return this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500; // FIXME
  }

  public boolean nearExtensionMinimum() {
    return this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500; // FIXME
  }

  private double getExtensionSetpoint() {
    return extensionSetpoint;
  }

  private double getRotationSetpoint() {
    return rotationSetpoint;
  }
}
