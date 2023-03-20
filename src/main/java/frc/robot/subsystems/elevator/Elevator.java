package frc.robot.subsystems.elevator;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  // FIXME: delete after tuning
  private final TunableNumber rotationPower = new TunableNumber("ElevatorRotation/power", 0.0);
  private final TunableNumber rotationPositionRadians =
      new TunableNumber("ElevatorRotation/radians", 0.0);
  private final TunableNumber extensionPower = new TunableNumber("ElevatorExtension/power", 0.0);
  private final TunableNumber extensionPositionMeters =
      new TunableNumber("ElevatorExtension/meters", 0.0);

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double rotationSetpoint = 0.0;
  private double extensionSetpoint = 0.0;
  private boolean extensionIsIncreasing;
  private boolean rotationIsIncreasing;
  private boolean toggledToCone = true;
  private boolean manualControlEnabled = false;
  private boolean manualPresetEnabled = false;
  private ElevatorIO io;
  private boolean reachedExtensionSetpoint = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    tabMain
        .addBoolean("Manual Control Enabled", this::isManualControlEnabled)
        .withPosition(0, 1)
        .withSize(2, 1);
    tabMain
        .addBoolean("Preset Enabled", this::isManualPresetEnabled)
        .withPosition(0, 2)
        .withSize(2, 1);
    // get the default instance of NetworkTables

    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add("elevator", this);
      tab.addNumber("Encoder", this::getExtensionElevatorEncoderHeight);
      tab.addNumber("Angle", this::getRotationElevatorEncoderAngle);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);

    Logger.getInstance().recordOutput("Elevator/atSetpoint", isAtSetpoint());
    Logger.getInstance().recordOutput("Elevator/toggledToCone", getToggledToCone());
  }

  public void setElevatorExtensionMotorPower(double power) {
    if (this.isManualControlEnabled()) {
      io.setExtensionMotorPercentage(power * MAX_MANUAL_POWER_EXTENSION);
    }
  }

  public void setElevatorRotationMotorPower(double power) {
    if (this.isManualControlEnabled()) {
      io.setRotationMotorPercentage(power * MAX_MANUAL_POWER_ROTATION);
    }
  }

  public boolean atRotationSetpoint() {
    // storage position is a special case: if the rotation angle is greater than the setpoint,
    // which it will be when we stall against the mechanical hard stop, we still want to
    // return that we are at the setpoint
    if (this.rotationSetpoint == Units.degreesToRadians(90.0 - 24.173)) {
      return this.inputs.rotationPositionRadians > this.rotationSetpoint;
    } else {
      return Math.abs(this.inputs.rotationPositionRadians - rotationSetpoint)
          < ELEVATOR_ROTATION_POSITION_TOLERANCE;
    }
  }

  public boolean atExtension(double targetExtension) {
    return Math.abs(this.inputs.extensionPositionMeters - targetExtension)
        < ELEVATOR_EXTENSION_POSITION_TOLERANCE;
  }

  public boolean atRotation(double targetRotation) {
    return Math.abs(this.inputs.rotationPositionRadians - targetRotation)
        < ELEVATOR_ROTATION_POSITION_TOLERANCE;
  }

  public boolean isAtSetpoint() {
    return io.isAtSetpoint();
  }

  public void stopElevator() {
    this.io.setExtensionMotorPercentage(0.0);
    this.io.setRotationMotorPercentage(0.0);
  }

  public double getExtensionElevatorEncoderHeight() {
    return this.inputs.extensionPositionMeters;
  }

  public double getRotationElevatorEncoderAngle() {
    return this.inputs.rotationPositionRadians;
  }

  public void initializePosition(double rotation, double extension) {
    this.extensionIsIncreasing = extension > this.getExtensionElevatorEncoderHeight();
    this.rotationIsIncreasing = rotation > this.getRotationElevatorEncoderAngle();
    this.reachedExtensionSetpoint = false;
  }

  public void setPosition(
      double rotation,
      double rotationCruiseVelocity,
      double rotationAcceleration,
      double extension,
      double extensionCruiseVelocity,
      double extensionAcceleration,
      double rotationExtensionTimeOffset,
      boolean applyTimeOffsetAtStart) {
    this.extensionSetpoint = extension;
    this.rotationSetpoint = rotation;

    io.setPosition(
        rotation,
        rotationCruiseVelocity,
        rotationAcceleration,
        extension,
        extensionCruiseVelocity,
        extensionAcceleration,
        rotationExtensionTimeOffset,
        applyTimeOffsetAtStart);
  }

  public void autoZeroExtension() {
    this.io.autoZeroExtension();
  }

  public boolean getToggledToCone() {
    return this.toggledToCone;
  }

  public void toggleToCone() {
    this.toggledToCone = true;
  }

  public void toggleToCube() {
    this.toggledToCone = false;
  }

  public boolean isManualControlEnabled() {
    return this.manualControlEnabled;
  }

  public void enableManualControl() {
    this.manualControlEnabled = true;
  }

  public void disableManualControl() {
    this.manualControlEnabled = false;
  }

  public boolean isManualPresetEnabled() {
    return this.manualPresetEnabled;
  }

  public void enableManualPreset() {
    this.manualPresetEnabled = true;
  }

  public void disableManualPreset() {
    this.manualPresetEnabled = false;
  }

  private double getExtensionSetpoint() {
    return extensionSetpoint;
  }

  private double getRotationSetpoint() {
    return rotationSetpoint;
  }

  private double timeToSetpoint() {
    return 0;
  }
}
