package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import frc.lib.team254.drivers.TalonFXFactory;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivers.SparkMAXFactory;
import frc.lib.team3061.swerve.Conversions;
import frc.lib.team3061.util.CANDeviceFinder;
import frc.lib.team3061.util.CANDeviceId.CANDeviceType;
import frc.lib.team6328.util.TunableNumber;

/*
 * Use factory classes for neo550 and falcon500
 * neo550 = roller motor
 * falcon500 = rotation motor
 * reference elevator branch for implementation of falcon500 using
 */

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX rotationMotor;
  private CANSparkMax rollerMotor;

  private final TunableNumber rkP = new TunableNumber("IntakeRotation/kP", ROTATION_POSITION_PID_P);
  private final TunableNumber rkI = new TunableNumber("IntakeRotation/kI", ROTATION_POSITION_PID_I);
  private final TunableNumber rkD = new TunableNumber("IntakeRotation/kD", ROTATION_POSITION_PID_D);
  private final TunableNumber rkF = new TunableNumber("IntakeRotation/kF", ROTATION_POSITION_PID_F);
  private final TunableNumber rkIz =
      new TunableNumber("IntakeRotation/kIz", ROTATION_POSITION_PID_I_ZONE);
  private final TunableNumber rkPeakOutput =
      new TunableNumber("IntakeRotation/kPeakOutput", ROTATION_POSITION_PID_PEAK_OUTPUT);

  public IntakeIOTalonFX() {
    CANDeviceFinder can = new CANDeviceFinder();

    can.isDevicePresent(CANDeviceType.TALON, INTAKE_ROTATION_MOTOR_CAN_ID, "Rotation Motor");
    can.isDevicePresent(CANDeviceType.SPARK_MAX, INTAKE_ROLLER_MOTOR_CAN_ID, "Roller Motor");

    // config motors
    configIntakeMotor(INTAKE_ROTATION_MOTOR_CAN_ID, INTAKE_ROLLER_MOTOR_CAN_ID);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rotationPositionDeg =
        Conversions.falconToDegrees(
            rotationMotor.getSelectedSensorPosition(SLOT_INDEX), INTAKE_ROTATION_GEAR_RATIO);
    inputs.rotationVelocityDegPerSec =
        Conversions.falconToDegrees(
            rotationMotor.getSelectedSensorVelocity(SLOT_INDEX), INTAKE_ROTATION_GEAR_RATIO);

    inputs.rotationPower = rotationMotor.getMotorOutputPercent();
    inputs.rotationClosedLoopError = rotationMotor.getClosedLoopError(SLOT_INDEX);
    inputs.rotationAppliedVolts = rotationMotor.getMotorOutputVoltage();
    inputs.rotationCurrentAmps = new double[] {rotationMotor.getStatorCurrent()};
    inputs.rotationTempCelsius = new double[] {rotationMotor.getTemperature()};

    inputs.rollerAppliedVolts = rollerMotor.getAppliedOutput();
    inputs.rollerCurrentAmps = new double[] {rollerMotor.getOutputCurrent()};
    inputs.rollerTempCelsius = new double[] {rollerMotor.getMotorTemperature()};

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
  public void setRotationMotorPercentage(double percentage) {
    rotationMotor.set(ControlMode.PercentOutput, percentage);
  }

  // FIXME: we may not need an abritrary feedforward and can just set it in the PIDF constants
  @Override
  public void setRotationPosition(double position, double arbitraryFeedForward) {
    rotationMotor.set(
        TalonFXControlMode.Position,
        Conversions.degreesToFalcon(position, INTAKE_ROTATION_GEAR_RATIO),
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  public void setRollerMotorPercentage(double percentage) {
    rollerMotor.set(percentage);
  }

  @Override
  public void resetRotationEncoder(double position) {
    rotationMotor.setSelectedSensorPosition(
        Conversions.degreesToFalcon(position, INTAKE_ROTATION_GEAR_RATIO));
  }

  private void configIntakeMotor(int intakeRotationMotorID, int intakeRollerMotorID) {
    TalonFXFactory.Configuration intakeRotationMotorConfig = new TalonFXFactory.Configuration();
    intakeRotationMotorConfig.INVERTED = IntakeConstants.INTAKE_ROTATION_MOTOR_INVERTED;
    intakeRotationMotorConfig.BRUSHLESS_CURRENT_STATUS_FRAME_RATE_MS = 9;
    intakeRotationMotorConfig.NEUTRAL_MODE = NeutralMode.Brake;
    intakeRotationMotorConfig.SLOT0_KP = rkP.get();
    intakeRotationMotorConfig.SLOT0_KI = rkI.get();
    intakeRotationMotorConfig.SLOT0_KD = rkD.get();
    intakeRotationMotorConfig.SLOT0_KF = rkF.get();
    // FIXME: why so low?
    intakeRotationMotorConfig.NEUTRAL_DEADBAND = 0.001;

    intakeRotationMotorConfig.STATOR_CURRENT_LIMIT = INTAKE_ROTATION_CURRENT_LIMIT;
    intakeRotationMotorConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 19;

    // limit rotation between 0 and 180 degrees
    intakeRotationMotorConfig.FORWARD_SOFT_LIMIT =
        (int) Conversions.degreesToFalcon(0.0, INTAKE_ROTATION_GEAR_RATIO);
    intakeRotationMotorConfig.REVERSE_SOFT_LIMIT =
        (int) Conversions.degreesToFalcon(180.0, INTAKE_ROTATION_GEAR_RATIO);
    // intakeRotationMotorConfig.ENABLE_SOFT_LIMIT = true;

    rotationMotor =
        TalonFXFactory.createTalon(
            intakeRotationMotorID,
            RobotConfig.getInstance().getCANBusName(),
            intakeRotationMotorConfig);

    rotationMotor.setSelectedSensorPosition(IntakeConstants.INTAKE_ROTATION_DEFAULT_POSITION);
    rotationMotor.configClosedLoopPeakOutput(0, rkPeakOutput.get());
    rotationMotor.config_IntegralZone(0, rkIz.get());
    rotationMotor.set(TalonFXControlMode.PercentOutput, 0);

    SparkMAXFactory.Configuration intakeRollerMotorConfig = new SparkMAXFactory.Configuration();
    intakeRollerMotorConfig.INVERTED = IntakeConstants.INTAKE_ROLLER_MOTOR_INVERTED;
    intakeRollerMotorConfig.IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    intakeRollerMotorConfig.SMART_STALL_CURRENT_LIMIT = IntakeConstants.INTAKE_ROLLER_CURRENT_LIMIT;
    rollerMotor = SparkMAXFactory.createSparkMax(intakeRollerMotorID, intakeRollerMotorConfig);
  }
}
