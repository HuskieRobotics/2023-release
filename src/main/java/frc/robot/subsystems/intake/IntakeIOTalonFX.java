package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.lib.team254.drivers.TalonFXFactory;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.Conversions;
import frc.lib.team6328.util.TunableNumber;

/*
 * Use factory classes for neo550 and falcon500
 * falcon500 = roller motor
 * falcon500 = rotation motor
 * reference elevator branch for implementation of falcon500 using
 */

public class IntakeIOTalonFX implements IntakeIO {
  private int stallCount;
  private TalonFX rotationMotor;
  private TalonFX rollerMotor;
  private int rollerStallCount;
  private int deployStallCount;

  private final TunableNumber rkP = new TunableNumber("IntakeRotation/kP", ROTATION_POSITION_PID_P);
  private final TunableNumber rkI = new TunableNumber("IntakeRotation/kI", ROTATION_POSITION_PID_I);
  private final TunableNumber rkD = new TunableNumber("IntakeRotation/kD", ROTATION_POSITION_PID_D);
  private final TunableNumber rkF = new TunableNumber("IntakeRotation/kF", ROTATION_POSITION_PID_F);
  private final TunableNumber rkIz =
      new TunableNumber("IntakeRotation/kIz", ROTATION_POSITION_PID_I_ZONE);
  private final TunableNumber rkPeakOutput =
      new TunableNumber("IntakeRotation/kPeakOutput", ROTATION_POSITION_PID_PEAK_OUTPUT);

  public IntakeIOTalonFX() {
    configIntakeMotor(INTAKE_ROTATION_MOTOR_CAN_ID, INTAKE_ROLLER_MOTOR_CAN_ID);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rotationPositionDeg =
        Conversions.falconToDegrees(
            rotationMotor.getSelectedSensorPosition(SLOT_INDEX), INTAKE_ROTATION_GEAR_RATIO);
    inputs.rotationPower = rotationMotor.getMotorOutputPercent();
    inputs.rotationAppliedPercentage = rotationMotor.getMotorOutputVoltage();
    inputs.rotationStatorCurrentAmps = new double[] {rotationMotor.getStatorCurrent()};
    inputs.rotationSupplyCurrent = new double[] {rotationMotor.getSupplyCurrent()};
    inputs.rollerAppliedPercentage = rollerMotor.getMotorOutputVoltage();
    inputs.rollerStatorCurrentAmps = new double[] {rollerMotor.getStatorCurrent()};
    inputs.rotationClosedLoopError = rotationMotor.getClosedLoopError();
    inputs.rotationSetpoint = rotationMotor.getClosedLoopTarget();

    if (inputs.rollerAppliedPercentage > 0
        && Math.abs(inputs.rollerStatorCurrentAmps[0]) > IntakeConstants.ROLLER_THRESHOLD_CURRENT) {
      rollerStallCount++;
      if (rollerStallCount > IntakeConstants.ROLLER_THRESHOLD_ITERATIONS) {
        inputs.atRollerCurrentThreshold = true;
        this.setRollerMotorPercentage(0.0);
      }
    } else {
      rollerStallCount = 0;
      inputs.atRollerCurrentThreshold = false;
    }

    if (inputs.rotationAppliedPercentage > 0
        && Math.abs(inputs.rotationStatorCurrentAmps[0])
            > IntakeConstants.ROTATION_THRESHOLD_CURRENT
        && inputs.rotationPositionDeg > 40) {
      deployStallCount++;
      if (deployStallCount > IntakeConstants.DEPLOY_THRESHOLD_ITERATIONS) {
        inputs.isDeployed = true;
        this.setRotationMotorPercentage(0.0);
      }
    } else if (inputs.rotationAppliedPercentage < 0) {
      deployStallCount = 0;
      inputs.isDeployed = false;
    } else {
      deployStallCount = 0;
    }

    if (inputs.isDeployed) {
      rotationMotor.setNeutralMode(NeutralMode.Coast);
    }

    if (inputs.rotationAppliedPercentage < 0
        && Math.abs(inputs.rotationStatorCurrentAmps[0])
            > IntakeConstants.ROTATION_THRESHOLD_CURRENT
        && inputs.rotationPositionDeg < 10) {
      stallCount++;
      if (stallCount > IntakeConstants.OPEN_THRESHOLD_ITERATIONS) {
        inputs.isRetracted = true;
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.set(ControlMode.Current, -20.0);
        rotationMotor.setSelectedSensorPosition(0.0);
      }
    } else if (inputs.rotationAppliedPercentage > 0) {
      stallCount = 0;
      inputs.isRetracted = false;
    } else {
      stallCount = 0;
    }

    if (rkP.hasChanged()
        || rkI.hasChanged()
        || rkD.hasChanged()
        || rkF.hasChanged()
        || rkIz.hasChanged()
        || rkPeakOutput.hasChanged()) {
      rotationMotor.config_kF(0, rkF.get());
      rotationMotor.config_kP(0, rkP.get());
      rotationMotor.config_kI(0, rkI.get());
      rotationMotor.config_kD(0, rkD.get());
      rotationMotor.config_IntegralZone(0, rkIz.get());
      rotationMotor.configClosedLoopPeakOutput(0, rkPeakOutput.get());
    }
  }

  @Override
  public void setRotationPosition(double position, double arbitraryFeedForward) {
    rotationMotor.setNeutralMode(NeutralMode.Brake);
    rotationMotor.set(
        TalonFXControlMode.Position,
        Conversions.degreesToFalcon(position, INTAKE_ROTATION_GEAR_RATIO),
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  public void setRotationMotorPercentage(double percentage) {
    rotationMotor.set(ControlMode.PercentOutput, percentage);
  }

  @Override
  public void setRollerMotorPercentage(double percentage) {
    rollerMotor.set(ControlMode.PercentOutput, percentage);
  }

  private void configIntakeMotor(int intakeRotationMotorID, int intakeRollerMotorID) {

    TalonFXFactory.Configuration intakeRotationMotorConfig = new TalonFXFactory.Configuration();
    intakeRotationMotorConfig.INVERTED = IntakeConstants.INTAKE_ROTATION_MOTOR_INVERTED;
    intakeRotationMotorConfig.BRUSHLESS_CURRENT_STATUS_FRAME_RATE_MS = 9;
    intakeRotationMotorConfig.NEUTRAL_MODE = NeutralMode.Brake;
    intakeRotationMotorConfig.STATOR_CURRENT_LIMIT = INTAKE_ROTATION_CURRENT_LIMIT;
    intakeRotationMotorConfig.SLOT0_KP = rkP.get();
    intakeRotationMotorConfig.SLOT0_KI = rkI.get();
    intakeRotationMotorConfig.SLOT0_KD = rkD.get();
    intakeRotationMotorConfig.SLOT0_KF = rkF.get();
    intakeRotationMotorConfig.NEUTRAL_DEADBAND = 0.001;
    rotationMotor =
        TalonFXFactory.createTalon(
            intakeRotationMotorID,
            RobotConfig.getInstance().getCANBusName(),
            intakeRotationMotorConfig);
    rotationMotor.setSelectedSensorPosition(0);
    rotationMotor.configClosedLoopPeakOutput(0, rkPeakOutput.get());
    rotationMotor.config_IntegralZone(0, rkIz.get());
    rotationMotor.set(ControlMode.Current, -20.0);

    TalonFXFactory.Configuration intakeRollerMotorConfig = new TalonFXFactory.Configuration();
    intakeRollerMotorConfig.INVERTED = IntakeConstants.INTAKE_ROLLER_MOTOR_INVERTED;
    intakeRollerMotorConfig.BRUSHLESS_CURRENT_STATUS_FRAME_RATE_MS = 9;
    intakeRollerMotorConfig.NEUTRAL_MODE = NeutralMode.Brake;
    intakeRollerMotorConfig.STATOR_CURRENT_LIMIT = IntakeConstants.INTAKE_ROLLER_CURRENT_LIMIT;
    rollerMotor =
        TalonFXFactory.createTalon(
            intakeRollerMotorID,
            RobotConfig.getInstance().getCANBusName(),
            intakeRollerMotorConfig);
  }
}
