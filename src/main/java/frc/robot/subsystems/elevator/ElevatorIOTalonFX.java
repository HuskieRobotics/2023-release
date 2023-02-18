package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import edu.wpi.first.math.util.Units;
import frc.lib.team254.drivers.TalonFXFactory;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.Conversions;
import frc.lib.team3061.util.CANDeviceFinder;
import frc.lib.team3061.util.CANDeviceId.CANDeviceType;
import frc.lib.team6328.util.TunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX extensionMotor;
  private TalonFX rotationMotor;
  private final String canBusName = RobotConfig.getInstance().getCANBusName();
  private Pigeon2 pigeon;

  private final TunableNumber rkP =
      new TunableNumber("ElevatorRotation/kP", ROTATION_POSITION_PID_P);
  private final TunableNumber rkI =
      new TunableNumber("ElevatorRotation/kI", ROTATION_POSITION_PID_I);
  private final TunableNumber rkD =
      new TunableNumber("ElevatorRotation/kD", ROTATION_POSITION_PID_D);
  private final TunableNumber rkPeakOutput =
      new TunableNumber("ElevatorRotation/kPeakOutput", ROTATION_POSITION_PID_PEAK_OUTPUT);

  private final TunableNumber ekP =
      new TunableNumber("ElevatorExtension/kP", EXTENSION_POSITION_PID_P);
  private final TunableNumber ekI =
      new TunableNumber("ElevatorExtension/kI", EXTENSION_POSITION_PID_I);
  private final TunableNumber ekD =
      new TunableNumber("ElevatorExtension/kD", EXTENSION_POSITION_PID_D);
  private final TunableNumber ekPeakOutput =
      new TunableNumber("ElevatorExtension/kPeakOutput", EXTENSION_POSITION_PID_PEAK_OUTPUT);

  public ElevatorIOTalonFX() {
    CANDeviceFinder can = new CANDeviceFinder();
    can.isDevicePresent(CANDeviceType.TALON, ELEVATOR_MOTOR_CAN_ID, "Elevator Extension");
    can.isDevicePresent(CANDeviceType.TALON, ROTATION_ELEVATOR_MOTOR_CAN_ID, "Elevator Rotation");

    TalonFXFactory.Configuration extensionConfig = new TalonFXFactory.Configuration();
    TalonFXFactory.Configuration rotationConfig = new TalonFXFactory.Configuration();

    extensionConfig.INVERTED = EXTENSION_INVERTED;
    extensionConfig.NEUTRAL_MODE = NeutralMode.Brake;
    extensionConfig.SLOT0_KP = ekP.get();
    extensionConfig.SLOT0_KI = ekI.get();
    extensionConfig.SLOT0_KD = ekD.get();

    rotationConfig.INVERTED = ROTATION_INVERTED;
    rotationConfig.NEUTRAL_MODE = NeutralMode.Brake;
    rotationConfig.SLOT0_KP = rkP.get();
    rotationConfig.SLOT0_KI = rkI.get();
    rotationConfig.SLOT0_KD = rkD.get();

    extensionConfig.STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 30, 32, 0.5);
    rotationConfig.STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 30, 32, 0.5);

    extensionConfig.MOTION_MAGIC_STATUS_FRAME_RATE_MS = 10;
    rotationConfig.MOTION_MAGIC_STATUS_FRAME_RATE_MS = 10;

    rotationConfig.REMOTE_SENSOR_DEVICE_ID = PIGEON_ID;
    rotationConfig.REMOTE_SENSOR_SOURCE = RemoteSensorSource.Pigeon_Pitch;

    extensionConfig.MOTION_ACCELERATION =
        Conversions.mpsToFalcon(
            EXTENSION_ELEVATOR_ACCELERATION, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO);
    final TunableNumber extensionConMotorAcceleration =
        new TunableNumber("extensionConMotorAcceleration", extensionConfig.MOTION_ACCELERATION);

    extensionConfig.MOTION_CRUISE_VELOCITY =
        Conversions.mpsToFalcon(
            EXTENSION_MAX_ELEVATOR_VELOCITY, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO);
    final TunableNumber extensionConMotorVelocity =
        new TunableNumber("extensionConMotorVelocity", extensionConfig.MOTION_CRUISE_VELOCITY);

    extensionConfig.MOTION_CURVE_STRENGTH = EXTENSION_SCURVE_STRENGTH;
    final TunableNumber extensionMotionCurveStrength =
        new TunableNumber("extensionMotionCurveStrength", extensionConfig.MOTION_CURVE_STRENGTH);

    rotationConfig.MOTION_ACCELERATION = radiansToPigeon(ROTATION_ELEVATOR_ACCELERATION);
    final TunableNumber rotationMotionAcceleration =
        new TunableNumber("rotationMotionAcceleration", rotationConfig.MOTION_ACCELERATION);

    rotationConfig.MOTION_CRUISE_VELOCITY = radiansToPigeon(ROTATION_MAX_ELEVATOR_VELOCITY);
    final TunableNumber rotationMotionVelocity =
        new TunableNumber("rotationMotionVelocity", rotationConfig.MOTION_CRUISE_VELOCITY);

    rotationConfig.MOTION_CURVE_STRENGTH = ROTATION_SCURVE_STRENGTH;
    final TunableNumber rotationMotionCurveStrength =
        new TunableNumber("rotationMotionCurveStrength", rotationConfig.MOTION_CURVE_STRENGTH);

    extensionMotor = TalonFXFactory.createTalon(ELEVATOR_MOTOR_CAN_ID, canBusName, extensionConfig);
    rotationMotor =
        TalonFXFactory.createTalon(ROTATION_ELEVATOR_MOTOR_CAN_ID, canBusName, rotationConfig);

    rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

    extensionMotor.configClosedLoopPeakOutput(SLOT_INDEX, ekPeakOutput.get());
    rotationMotor.configClosedLoopPeakOutput(SLOT_INDEX, rkPeakOutput.get());

    /* Initialize */
    this.rotationMotor.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT_MS);

    // FIXME: initialie the extension sensor to a constant which represents the starting position of
    // the carriage when holding a cone

    this.pigeon = new Pigeon2(PIGEON_ID);
    Pigeon2Configuration config = new Pigeon2Configuration();
    // set mount pose as rolled 90 degrees counter-clockwise and pitched 90 degrees clockwise
    config.MountPoseYaw = 0;
    config.MountPosePitch = -90.0;
    config.MountPoseRoll = 90.0;
    this.pigeon.configAllSettings(config);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.extensionSetpointMeters =
        Conversions.falconToMeters(
            extensionMotor.getClosedLoopTarget(),
            EXTENSION_PULLEY_CIRCUMFERENCE,
            EXTENSION_GEAR_RATIO);
    inputs.extensionPositionMeters =
        Conversions.falconToMeters(
            extensionMotor.getSelectedSensorPosition(SLOT_INDEX),
            EXTENSION_PULLEY_CIRCUMFERENCE,
            EXTENSION_GEAR_RATIO);
    inputs.extensionVelocityMetersPerSec =
        Conversions.falconToMPS(
            extensionMotor.getSelectedSensorVelocity(SLOT_INDEX),
            EXTENSION_PULLEY_CIRCUMFERENCE,
            EXTENSION_GEAR_RATIO);
    inputs.extensionClosedLoopErrorMeters =
        Conversions.falconToMeters(
            extensionMotor.getClosedLoopError(SLOT_INDEX),
            EXTENSION_PULLEY_CIRCUMFERENCE,
            EXTENSION_GEAR_RATIO);
    inputs.extensionAppliedVolts = extensionMotor.getMotorOutputVoltage();
    inputs.extensionCurrentAmps = new double[] {extensionMotor.getStatorCurrent()};
    inputs.extensionTempCelsius = new double[] {extensionMotor.getTemperature()};

    inputs.rotationSetpointRadians = pigeonToRadians(rotationMotor.getClosedLoopTarget(SLOT_INDEX));
    inputs.rotationPositionRadians =
        pigeonToRadians(rotationMotor.getSelectedSensorPosition(SLOT_INDEX));
    inputs.rotationVelocityRadiansPerSec =
        pigeonToRadians(rotationMotor.getSelectedSensorVelocity(SLOT_INDEX));
    inputs.rotationClosedLoopErrorRadians =
        pigeonToRadians(rotationMotor.getClosedLoopError(SLOT_INDEX));
    inputs.rotationAppliedVolts = rotationMotor.getMotorOutputVoltage();
    inputs.rotationCurrentAmps = new double[] {rotationMotor.getStatorCurrent()};
    inputs.rotationTempCelsius = new double[] {rotationMotor.getTemperature()};

    inputs.pitchRadians =
        Units.degreesToRadians(pigeon.getPitch()); // FIXME: verify Pigeon returns degrees

    // update tunables
    if (rkP.hasChanged() || rkI.hasChanged() || rkD.hasChanged() || rkPeakOutput.hasChanged()) {
      this.rotationMotor.config_kP(SLOT_INDEX, rkP.get());
      this.rotationMotor.config_kI(SLOT_INDEX, rkI.get());
      this.rotationMotor.config_kD(SLOT_INDEX, rkD.get());
      this.rotationMotor.configPeakOutputForward(rkPeakOutput.get());
      this.rotationMotor.configPeakOutputReverse(rkPeakOutput.get());
      this.rotationMotor.configClosedLoopPeakOutput(SLOT_INDEX, rkPeakOutput.get());
    }

    if (ekP.hasChanged() || ekI.hasChanged() || ekD.hasChanged() || ekPeakOutput.hasChanged()) {
      this.extensionMotor.config_kP(SLOT_INDEX, rkP.get());
      this.extensionMotor.config_kI(SLOT_INDEX, rkI.get());
      this.extensionMotor.config_kD(SLOT_INDEX, rkD.get());
      this.extensionMotor.configPeakOutputForward(rkPeakOutput.get());
      this.extensionMotor.configPeakOutputReverse(rkPeakOutput.get());
      this.extensionMotor.configClosedLoopPeakOutput(SLOT_INDEX, ekPeakOutput.get());
    }
  }

  private double pigeonToRadians(double counts) {
    return counts / PIGEON_UNITS_PER_ROTATION * (2 * Math.PI);
  }

  private double radiansToPigeon(double radians) {
    return radians / (2 * Math.PI) * PIGEON_UNITS_PER_ROTATION;
  }

  @Override
  public void setExtensionMotorPercentage(double percentage) {
    extensionMotor.set(ControlMode.PercentOutput, percentage);
  }

  @Override
  public void setRotationMotorPercentage(double percentage) {
    rotationMotor.set(ControlMode.PercentOutput, percentage);
  }

  @Override
  public void setExtensionPosition(double position, double arbitraryFeedForward) {
    extensionMotor.set(
        TalonFXControlMode.MotionMagic,
        position,
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  public void setRotationPosition(double position, double arbitraryFeedForward) {
    rotationMotor.set(
        TalonFXControlMode.MotionMagic,
        position,
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }
}
