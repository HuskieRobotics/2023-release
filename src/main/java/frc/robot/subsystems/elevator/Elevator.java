package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  public double rotationSetpoint = 0.0;
  public double extensionSetpoint = 0.0;
  public double power = 0.0;

  public boolean isControlEnabled = false;

  public ElevatorIO io;
  public int valueListenerHandle;

  // private double extension = 0.0;
  // private double rotation = 0.0;

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

    if (TUNING) {
      io.setControlEnabled(true);

      tab.addNumber("Rotation Closed Loop Target", this::getExtensionSetpoint);
      tab.addNumber("Rotation Closed Loop Error", () -> inputs.rotationClosedLoopError);
      tab.addNumber("Rotation Velocity", () -> inputs.rotationVelocityRadiansPerSec);
      tab.addNumber("Rotation Right Motor Volts", () -> inputs.rotationAppliedVolts);

      tab.addNumber("Extension Closed Loop Target", this::getRotationSetpoint);
      tab.addNumber("Extension Closed Loop Error", () -> inputs.extensionClosedLoopError);
      tab.addNumber("Extension Velocity", () -> inputs.extensionVelocityMetersPerSec);
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
    if ((this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500 && power > 0)
        || (this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500
            && power < 0)) {
      stopExtension();
    } else {
      if (getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500) {
        io.setRotationPosition(
            MIN_ROTATION_POSITION + 2500,
            calculateRotationFeedForward(
                convertMetersToInches(inputs.extensionPositionMeters),
                inputs.rotationPositionRadians));
      }
      this.extensionSetpoint = extension;
      io.setExtensionPosition(
          extensionSetpoint,
          calculateExtensionFeedForward(
              convertMetersToInches(inputs.extensionPositionMeters),
              inputs.rotationPositionRadians));
    }
  }

  public void setElevatorRotation(Double rotation) {
    if ((this.getRotationElevatorEncoderAngle() > MAX_ROTATION_POSITION - 2500 && power > 0)
        || (this.getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500 && power < 0)) {
      this.stopExtension();
    } else {
      io.setRotationPosition(
          rotation,
          calculateRotationFeedForward(
              convertMetersToInches(inputs.extensionPositionMeters),
              inputs.rotationPositionRadians));
      this.rotationSetpoint = rotation;
    }
  }

  public boolean atExtensionSetpoint() {
    return Math.abs(this.inputs.extensionPositionMeters - extensionSetpoint)
        < ELEVATOR_EXTENSION_POSITION_TOLERANCE;
  }

  public boolean atRotationSetpoint() {
    return Math.abs(this.inputs.rotationPositionRadians - rotationSetpoint)
        < ELEVATOR_ROTATION_POSITION_TOLERANCE;
  }

  public boolean atSetpoint() {
    return (Math.abs(this.inputs.rotationPositionRadians - rotationSetpoint)
            < ELEVATOR_ROTATION_POSITION_TOLERANCE)
        && (Math.abs(this.inputs.extensionPositionMeters - extensionSetpoint)
            < ELEVATOR_EXTENSION_POSITION_TOLERANCE);
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

  public void setPosition(Double rotation, Double extension) {
    // if((INTAKE_STORED) && (getRotationElevatorEncoderAngle() < .48)){ // radians
    //   this.setElevatorExtension(extension);
    //   this.setElevatorRotation(rotation);
    // }
    // else if((INTAKE_STORED) && (getExtensionElevatorEncoderHeight() > .381)){// meters
    //   this.setElevatorExtension(extension);
    //   this.setElevatorRotation(rotation);
    // }
    // else if((INTAKE_STORED) && (getExtensionElevatorEncoderHeight() == 0) &&
    // (getRotationElevatorEncoderAngle() < .345)){ // 0 is in meters, .345 is radians
    //   this.setElevatorExtension(extension);
    //   this.setElevatorRotation(rotation);
    // }
    // else if((!INTAKE_STORED) && (getExtensionElevatorEncoderHeight() == 0) &&
    // (getRotationElevatorEncoderAngle() < .4)){  // radians
    //   this.setElevatorExtension(extension);
    //   this.setElevatorRotation(rotation);
    // }
    // else if((!INTAKE_STORED) && (getExtensionElevatorEncoderHeight() > .686) &&
    // (getRotationElevatorEncoderAngle() < .478)){ // .686meters, .478 radians
    //   this.setElevatorExtension(extension);
    //   this.setElevatorRotation(rotation);
    // }
    // else if((!INTAKE_STORED) && (getRotationElevatorEncoderAngle() < .478) &&
    // (getExtensionElevatorEncoderHeight() > .686)){// .478meter, .686radians
    //   this.setElevatorExtension(extension);
    //   this.setElevatorRotation(rotation);
    // }
    // else if((!INTAKE_STORED) && (getRotationElevatorEncoderAngle() > .478)){ // FIXME if we are
    // careful to position the intake such that its hood is collapsed by the elevator
    //   this.setElevatorExtension(extension);
    //   this.setElevatorRotation(rotation);
    // }
    // else if((!INTAKE_STORED) && (getRotationElevatorEncoderAngle() < .44) &&
    // (getExtensionElevatorEncoderHeight() > .305)){ // .44radians, .305meters
    //   this.setElevatorExtension(extension);
    //   this.setElevatorRotation(rotation);
    // }
    this.setElevatorExtension(extension);
    this.setElevatorRotation(rotation);
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
      mass = CARRIAGE_MASS + MOVING_STAGE_MASS;
    }

    double f = mass * Math.cos(rotation);

    return (MIN_MOTOR_POWER_TO_EXTEND_CARRIAGE / CARRIAGE_MASS) * f;
  }

  private static double convertMetersToInches(double meters) {
    return meters * 39.3701;
  }
}
