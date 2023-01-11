package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final TunableNumber rotationPower = new TunableNumber("IntakeRotation/power", 0.0);
  private final TunableNumber rotationPosition = new TunableNumber("IntakeRotation/rotation", 0.0);
  private final TunableNumber rollerPower = new TunableNumber("IntakeRoller/power", 0.0);

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeIO io;
  private double rotationSetpoint;

  public Intake(IntakeIO io) {

    this.io = io;
    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add("Intake", this);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

    if (TESTING) {
      if (rotationPosition.get() != 0) {
        this.setRotationPosition(rotationPosition.get());
      }

      if (rotationPower.get() != 0) {
        this.setRotationMotorPercentage(rotationPower.get());
      }

      if (rollerPower.get() != 0) {
        this.setRollerMotorPercentage(rollerPower.get());
      }
    }
  }

  public void deploy() {
    this.setRotationMotorPercentage(IntakeConstants.DEPLOY_POWER);
  }

  public void retract() {
    this.setRotationMotorPercentage(IntakeConstants.RETRACT_POWER);
  }

  public void stopRotation() {
    this.setRotationMotorPercentage(0);
  }

  public void enableRoller() {
    this.setRollerMotorPercentage(IntakeConstants.INTAKE_DEFAULT_ROLLER_POWER);
  }

  public void disableRoller() {
    this.setRollerMotorPercentage(0);
  }

  public boolean isDeployed() {
    return inputs.isDeployed;
  }

  public boolean isRetracted() {
    return inputs.isRetracted;
  }

  public void setRotationMotorPercentage(double power) {
    io.setRotationMotorPercentage(power);
  }

  public void setRollerMotorPercentage(double power) {
    io.setRollerMotorPercentage(power);
  }

  public void setRotationPosition(double position) {
    this.rotationSetpoint = position;
    io.setRotationPosition(position, IntakeConstants.ROTATION_FEEDFORWARD);
  }

  public void stopIntake() {
    this.disableRoller();
    this.stopRotation();
  }

  public boolean isAtRotationSetpoint() {
    return Math.abs(this.rotationSetpoint - this.getRotationMotorPosition())
        < INTAKE_ROTATION_TOLERANCE;
  }

  public double getRotationMotorPosition() {
    return inputs.rotationPositionDeg;
  }

  public boolean hasGamePiece() {
    return inputs.atRollerCurrentThreshold;
  }

  public boolean isIntakeRollerSpinning() {
    if (inputs.rotationAppliedVoltage > 0) {
      return true;
    } else {
      return false;
    }
  }
}
