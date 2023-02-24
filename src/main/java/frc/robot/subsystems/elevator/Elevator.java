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
  private ElevatorIO io;

  public Elevator(ElevatorIO io) {
    this.io = io;
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    // get the default instance of NetworkTables

    if (DEBUGGING) {
      tab.add("elevator", this);
      tab.addNumber("Encoder", this::getExtensionElevatorEncoderHeight);
      tab.addNumber("Angle", this::getRotationElevatorEncoderAngle);
    }

    if (TESTING) {}
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);

    // FIXME: update feedforward to call methods once elevator is assembled
    if (TESTING) {
      if (rotationPositionRadians.get() != 0) {
        io.setRotationPosition(rotationPositionRadians.get(), 0.0);
      }

      if (rotationPower.get() != 0) {
        io.setRotationMotorPercentage(rotationPower.get());
      }

      if (extensionPositionMeters.get() != 0) {
        io.setExtensionPosition(extensionPositionMeters.get(), 0.0);
      }

      if (extensionPower.get() != 0) {
        io.setExtensionMotorPercentage(extensionPower.get());
      }
    }

    /*
    if (extension < 69) {
      double elevatorF = calculateExtensionFeedForward(extension, rotation);
      double armF = calculateRotationFeedForward(extension, rotation);
      Logger.getInstance().recordOutput("Elevator/extension", extension);
      Logger.getInstance().recordOutput("Elevator/rotation", rotation);
      Logger.getInstance().recordOutput("Elevator/elevatorF", elevatorF);
      Logger.getInstance().recordOutput("Elevator/armF", armF);

      rotation += 0.05;
      if (rotation > Math.PI / 2) {
        rotation = 0.0;
        extension += 2.0;
      }
    }
    */
  }

  public void setElevatorExtensionMotorPower(double power) {
    // if ((this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500 && power > 0)
    //     || (this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500
    //         && power < 0)) {
    //   stopExtension();
    // } else
    {
      io.setExtensionMotorPercentage(power);
    }
  }

  public void setElevatorRotationMotorPower(double power) {
    // if ((this.getRotationElevatorEncoderAngle() > MAX_ROTATION_POSITION - 2500 && power > 0)
    //     || (this.getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500 && power < 0))
    // {
    //   stopRotation();
    // } else
    {
      io.setRotationMotorPercentage(power);
    }
  }

  public void setElevatorExtension(double extension) {
    this.extensionSetpoint = extension;
    io.setExtensionPosition(
        extension,
        calculateExtensionFeedForward(
            Units.metersToInches(inputs.extensionPositionMeters), inputs.rotationPositionRadians));
  }

  public void setElevatorRotation(Double rotation) {
    this.rotationSetpoint = rotation;
    io.setRotationPosition(
        rotation,
        calculateRotationFeedForward(
            Units.metersToInches(inputs.extensionPositionMeters), inputs.rotationPositionRadians));
  }

  public boolean atExtensionSetpoint() {
    return Math.abs(this.inputs.extensionPositionMeters - extensionSetpoint)
        < ELEVATOR_EXTENSION_POSITION_TOLERANCE;
  }

  public boolean atRotationSetpoint() {
    return Math.abs(this.inputs.rotationPositionRadians - rotationSetpoint)
        < ELEVATOR_ROTATION_POSITION_TOLERANCE;
  }

  public boolean atExtension(double targetExtension) {
    return Math.abs(this.inputs.extensionPositionMeters - targetExtension)
        < ELEVATOR_EXTENSION_POSITION_TOLERANCE;
  }

  public boolean atRotation(double targetRotation) {
    return Math.abs(this.inputs.rotationPositionRadians - targetRotation)
        < ELEVATOR_ROTATION_POSITION_TOLERANCE;
  }

  public boolean atSetpoint() {
    return atExtensionSetpoint() && atRotationSetpoint();
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
    return this.inputs.extensionPositionMeters;
  }

  public double getRotationElevatorEncoderAngle() {
    return this.inputs.rotationPositionRadians;
  }

  public boolean storageElevatorPosition() { // is the elevator in the storage position
    if (getExtensionElevatorEncoderHeight() == 0
        && getRotationElevatorEncoderAngle() == Units.degreesToRadians(20)) {
      return true;
    }
    return false;
  }

  public boolean coneFloorPosition() {
    if (getExtensionElevatorEncoderHeight() == Units.inchesToMeters(34)
        && getRotationElevatorEncoderAngle() == Units.degreesToRadians(82)) {
      return true;
    }
    return false;
  }

  public boolean cubeFloorPosition() {
    if (getExtensionElevatorEncoderHeight() == Units.inchesToMeters(8)
        && getRotationElevatorEncoderAngle() == Units.degreesToRadians(43)) {
      return true;
    }
    return false;
  }

  public void setPosition(double rotation, double extension, boolean intakeStored) {
    boolean extensionIsIncreasing = extension > this.getExtensionElevatorEncoderHeight();
    boolean rotationIsIncreasing = rotation > this.getRotationElevatorEncoderAngle();

    /*
     * We can't call atRotationSetpoint or atExtensionSetpoint because we haven't set
     * the new setpoint values yet. Instead, we will compare the current position to the eventual position.
     */

    if (extensionIsIncreasing && !rotationIsIncreasing) {
      if ((extension < Units.inchesToMeters(52.0))
          || (this.getRotationElevatorEncoderAngle() <= Units.degreesToRadians(90.0 - 48.0))
          || (this.atRotation(rotation))) {
        this.setElevatorExtension(extension);
      } else {
        this.setElevatorExtension(Units.inchesToMeters(52.0));
      }

      if ((this.getExtensionElevatorEncoderHeight() >= Units.inchesToMeters(22.0))
          || (this.atExtension(extension))) {
        this.setElevatorRotation(rotation);
      }
    } else if (!extensionIsIncreasing && rotationIsIncreasing) {
      if ((extension >= Units.inchesToMeters(22.0))
          || (this.getRotationElevatorEncoderAngle() >= Units.degreesToRadians(90.0 - 20.0))
          || (this.atRotation(rotation))) {
        this.setElevatorExtension(extension);
      } else {
        this.setElevatorExtension(Units.inchesToMeters(22.0));
      }

      if ((this.getExtensionElevatorEncoderHeight() <= Units.inchesToMeters(52.0))
          || (this.atExtension(extension))) {
        this.setElevatorRotation(rotation);
      }
    } else if (extensionIsIncreasing && rotationIsIncreasing) {
      if ((extension < Units.inchesToMeters(52.0))
          || (this.getRotationElevatorEncoderAngle() >= Units.degreesToRadians(90.0 - 48.0))
          || (this.atRotation(rotation))) {
        this.setElevatorExtension(extension);
      } else {
        this.setElevatorExtension(Units.inchesToMeters(52.0));
      }

      this.setElevatorRotation(rotation);
    } else if (!extensionIsIncreasing && !rotationIsIncreasing) {
      if ((this.getExtensionElevatorEncoderHeight() <= Units.inchesToMeters(52.0))
          || (this.atExtension(extension))) {
        this.setElevatorRotation(rotation);
      }

      this.setElevatorExtension(extension);
    }
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

  private double timeToSetpoint() {
    return 0;
  }

  private static final double D1 = 39.8;
  private static final double D2 = 40.3;
  private static final double D3 = 3.9;
  private static final double D5 = 40.5;
  private static final double H1 = 14.0;
  private static final double H2 = 49.0;
  private static final double M = 21.6;
  private static final double T_SPRING = 34.0;
  private static final double MAX_EXTENSION_BEFORE_MOVING_STAGE_ENGAGEMENT = 34.0;
  private static final double CARRIAGE_MASS = 8.682;
  private static final double MOVING_STAGE_MASS = 4.252;
  private static final double FIXED_STAGE_MASS = 9.223;
  private static final double F1 = 11.2;
  private static final double MIN_MOTOR_POWER_TO_EXTEND_CARRIAGE = 0.1; // FIXME: tune
  private static final double MIN_MOTOR_POWER_TO_ROTATE_COLLAPSED_ELEVATOR = 0.1; // FIXME: tune

  private static double calculateRotationFeedForward(double extension, double rotation) {
    double r =
        Math.sqrt(
            Math.pow((D2 - D1 * Math.sin(rotation)), 2)
                + Math.pow((D1 * Math.cos(rotation) + D3), 2));
    double Sa =
        Math.sqrt(
            1 - Math.pow((Math.pow(r, 2) + Math.pow(D1, 2) - Math.pow(D5, 2)) / (2 * D1 * r), 2));
    double h;

    if (extension <= MAX_EXTENSION_BEFORE_MOVING_STAGE_ENGAGEMENT) {
      h = 14.0 + 0.441176 * extension;
    } else {
      h = 0.575539 * extension + 9.43165;
    }

    double F3 =
        (M * h * Math.cos(rotation) + T_SPRING * ((2 * rotation / Math.PI) + 1.0 / 3.0))
            / (D1 * Sa);

    return (MIN_MOTOR_POWER_TO_ROTATE_COLLAPSED_ELEVATOR / F1) * F3;
  }

  private static double calculateExtensionFeedForward(double extension, double rotation) {
    double mass;
    if (extension <= MAX_EXTENSION_BEFORE_MOVING_STAGE_ENGAGEMENT) {
      mass = CARRIAGE_MASS;
    } else {
      mass = (CARRIAGE_MASS + MOVING_STAGE_MASS) / 2.0; // two belts are now in tension
    }

    double f = mass * Math.cos(rotation);

    return (MIN_MOTOR_POWER_TO_EXTEND_CARRIAGE / CARRIAGE_MASS) * f;
  }
}
