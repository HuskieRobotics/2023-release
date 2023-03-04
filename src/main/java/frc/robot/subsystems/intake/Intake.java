package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.commands.AutoZeroIntake;
import frc.robot.commands.SetIntakeState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final TunableNumber rotationPower = new TunableNumber("IntakeRotation/power", 0.0);
  private final TunableNumber rotationPositionDegrees =
      new TunableNumber("IntakeRotation/degrees", 0.0);
  private final TunableNumber rollerPower = new TunableNumber("IntakeRoller/power", 0.0);

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double rotationSetpoint;
  private IntakeIO io;

  public Intake(IntakeIO io) {

    this.io = io;
    this.rotationSetpoint = 0;

    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);

    if (TESTING) {
      tab.add("Intake", this);
      tab.add("Set Intake State", new SetIntakeState(this));
      tab.add("Auto Zero", new AutoZeroIntake(this));
    }

    if (TUNING) {
      tab.addNumber("Rotation Motor Position Degrees", () -> inputs.rotationPositionDeg);
      tab.addNumber("Rotation Motor Power", () -> inputs.rotationPower);
      tab.addNumber("Rotation Closed Loop Error", () -> inputs.rotationClosedLoopError);
      tab.addNumber("Rotation Velocity", () -> inputs.rotationVelocityDegPerSec);
      tab.addNumber("Rotation Motor Volts", () -> inputs.rotationAppliedVolts);

      tab.addNumber("Roller Motor Volts", () -> inputs.rollerAppliedVolts);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

    if (TESTING) {
      if (rotationPositionDegrees.get() != 0) {
        this.setRotationPosition(rotationPositionDegrees.get());
      }

      if (rotationPower.get() != 0) {
        this.setRotationMotorPercentage(rotationPower.get());
      }

      if (rollerPower.get() != 0) {
        this.setRollerMotorPercentage(rollerPower.get());
      }
    }
  }

  public void setRotationPosition(double position) {
    this.rotationSetpoint = position;
    io.setRotationPosition(position, IntakeConstants.ROTATION_FEEDFORWARD);
  }

  public double getRotationMotorPosition() {
    return inputs.rotationPositionDeg;
  }

  public void stopRotation() {
    this.setRotationMotorPercentage(0);
  }

  public void setRotationMotorPercentage(double power) {
    io.setRotationMotorPercentage(power);
  }

  public void setRollerMotorPercentage(double power) {
    io.setRollerMotorPercentage(power);
  }

  public void enableRoller() {
    io.setRollerMotorPercentage(IntakeConstants.INTAKE_DEFAULT_ROLLER_POWER);
  }

  public void stopRoller() {
    io.setRollerMotorPercentage(0);
  }

  public void stopIntake() {
    this.stopRoller();
    this.stopRotation();
  }

  public boolean atSetpoint() {
    return Math.abs(this.rotationSetpoint - this.getRotationMotorPosition())
        < INTAKE_ROTATION_TOLERANCE;
  }

  public boolean isRotationMotorPastCurrentLimit() {
    return Math.abs(inputs.rotationCurrentAmps[inputs.rotationCurrentAmps.length - 1])
        > INTAKE_ROTATION_CURRENT_THRESHOLD;
  }

  public boolean isRollerSpinning() {
    return inputs.rollerAppliedVolts > 0;
  }

  public void resetRotationEncoder(double position) {
    io.resetRotationEncoder(position);
  }
}
