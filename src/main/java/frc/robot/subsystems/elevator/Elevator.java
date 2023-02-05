package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  public double rotationSetpoint = 0.0;
  public double extensionSetpoint = 0.0;
  public double power = 0.0;

  public boolean isControlEnabled = false;

  public ElevatorIO io;
  public int valueListenerHandle;

  public Elevator(ElevatorIO io) {
    this.io = io;

    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    // get the default instance of NetworkTables

    if (DEBUGGING) {
      tab.add("elevator", this);
      tab.addNumber("Encoder", this::getExtensionElevatorEncoderHeight);
      tab.addNumber("Angle", this::getRotationElevatorEncoderAngle);
      tab.add("Extension", tab)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
          .getEntry();
    }

    if (TESTING) {}

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

      final TunableNumber extensionKf =
          new TunableNumber("Drive/DriveKp", RobotConfig.getInstance().getElevatorExtensionKP());
      final TunableNumber extensionKp =
          new TunableNumber("Drive/DriveKi", RobotConfig.getInstance().getSwerveDriveKI());
      final TunableNumber extensionKi =
          new TunableNumber("Drive/DriveKd", RobotConfig.getInstance().getSwerveDriveKD());
      final TunableNumber extensionKd =
          new TunableNumber("Drive/TurnKp", RobotConfig.getInstance().getSwerveAngleKP());

      final TunableNumber rotationKf =
          new TunableNumber("Drive/TurnKi", RobotConfig.getInstance().getSwerveAngleKI());
      final TunableNumber rotationKp =
          new TunableNumber("Drive/TurnKd", RobotConfig.getInstance().getSwerveAngleKD());
      final TunableNumber rotationKi =
          new TunableNumber("Drive/DriveKd", RobotConfig.getInstance().getSwerveDriveKD());
      final TunableNumber rotationKd =
          new TunableNumber("Drive/TurnKp", RobotConfig.getInstance().getSwerveAngleKP());
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
