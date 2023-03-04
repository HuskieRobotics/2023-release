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
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
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
  private int stallCount;
  private double extensionSetpoint = -1.0;

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
  private final TunableNumber ekF =
      new TunableNumber("ElevatorExtension/kF", EXTENSION_POSITION_PID_F);
  private final TunableNumber ekPeakOutput =
      new TunableNumber("ElevatorExtension/kPeakOutput", EXTENSION_POSITION_PID_PEAK_OUTPUT);

  private final TunableNumber extensionMagicMotionAcceleration =
      new TunableNumber(
          "ElevatorExtension/MMAcceleration",
          EXTENSION_ELEVATOR_ACCELERATION_METERS_PER_SECOND_PER_SECOND);
  private final TunableNumber extensionMagicMotionCruiseVelocity =
      new TunableNumber(
          "ElevatorExtension/MMVelocity", EXTENSION_MAX_ELEVATOR_VELOCITY_METERS_PER_SECOND);
  private final TunableNumber extensionMagicMotionSCurveStrength =
      new TunableNumber("ElevatorExtension/MMSCurve", EXTENSION_SCURVE_STRENGTH);

  public ElevatorIOTalonFX() {
    CANDeviceFinder can = new CANDeviceFinder();
    can.isDevicePresent(CANDeviceType.TALON, EXTENSION_ELEVATOR_MOTOR_CAN_ID, "Elevator Extension");
    can.isDevicePresent(CANDeviceType.TALON, ROTATION_ELEVATOR_MOTOR_CAN_ID, "Elevator Rotation");

    /* create and configure the Pigeon */
    this.pigeon = new Pigeon2(PIGEON_ID, RobotConfig.getInstance().getCANBusName());
    Pigeon2Configuration config = new Pigeon2Configuration();
    // set mount pose as rolled 90 degrees clockwise
    config.MountPoseYaw = 0;
    config.MountPoseRoll = -90.0;
    this.pigeon.configAllSettings(config);
    this.pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 9);

    TalonFXFactory.Configuration extensionConfig = new TalonFXFactory.Configuration();
    TalonFXFactory.Configuration rotationConfig = new TalonFXFactory.Configuration();

    extensionConfig.INVERTED = EXTENSION_INVERTED;
    extensionConfig.NEUTRAL_MODE = NeutralMode.Brake;
    extensionConfig.SLOT0_KP = ekP.get();
    extensionConfig.SLOT0_KI = ekI.get();
    extensionConfig.SLOT0_KD = ekD.get();
    extensionConfig.SLOT0_KF = ekF.get();

    rotationConfig.INVERTED = ROTATION_INVERTED;
    rotationConfig.NEUTRAL_MODE = NeutralMode.Brake;
    rotationConfig.SLOT0_KP = rkP.get();
    rotationConfig.SLOT0_KI = rkI.get();
    rotationConfig.SLOT0_KD = rkD.get();

    extensionConfig.STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 30, 32, 0.5);
    rotationConfig.STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 30, 32, 0.5);

    rotationConfig.REMOTE_SENSOR_DEVICE_ID = PIGEON_ID;
    rotationConfig.REMOTE_SENSOR_SOURCE = RemoteSensorSource.Pigeon_Pitch;
    rotationConfig.SENSOR_PHASE = true;

    // limit rotation between 0 and 90 degrees
    rotationConfig.FORWARD_SOFT_LIMIT = (int) radiansToPigeon(MAX_ROTATION_POSITION);
    rotationConfig.REVERSE_SOFT_LIMIT = (int) radiansToPigeon(MIN_ROTATION_POSITION);
    rotationConfig.ENABLE_SOFT_LIMIT = true;

    extensionConfig.MOTION_ACCELERATION =
        mpsToFalconMotionMagicUnits(
            extensionMagicMotionAcceleration.get(),
            EXTENSION_PULLEY_CIRCUMFERENCE,
            EXTENSION_GEAR_RATIO);
    extensionConfig.MOTION_CRUISE_VELOCITY =
        mpsToFalconMotionMagicUnits(
            extensionMagicMotionCruiseVelocity.get(),
            EXTENSION_PULLEY_CIRCUMFERENCE,
            EXTENSION_GEAR_RATIO);
    extensionConfig.MOTION_CURVE_STRENGTH = (int) extensionMagicMotionSCurveStrength.get();

    extensionConfig.MOTION_MAGIC_STATUS_FRAME_RATE_MS = 9;
    extensionConfig.BASE_PIDF0_STATUS_FRAME_RATE_MS = 9;

    rotationConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 9;
    rotationConfig.BASE_PIDF0_STATUS_FRAME_RATE_MS = 9;

    extensionMotor =
        TalonFXFactory.createTalon(EXTENSION_ELEVATOR_MOTOR_CAN_ID, canBusName, extensionConfig);
    rotationMotor =
        TalonFXFactory.createTalon(ROTATION_ELEVATOR_MOTOR_CAN_ID, canBusName, rotationConfig);

    rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    rotationMotor.config_IntegralZone(0, radiansToPigeon(ROTATION_POSITION_PID_I_ZONE));

    // FIXME: change to starting position when holding a cone
    extensionMotor.setSelectedSensorPosition(
        Conversions.metersToFalcon(
            Units.inchesToMeters(START_EXTENSION_POSITION_INCHES),
            EXTENSION_PULLEY_CIRCUMFERENCE,
            EXTENSION_GEAR_RATIO));

    extensionMotor.configClosedLoopPeakOutput(SLOT_INDEX, ekPeakOutput.get());
    extensionMotor.configPeakOutputForward(ekPeakOutput.get());
    extensionMotor.configPeakOutputReverse(-ekPeakOutput.get());

    rotationMotor.configClosedLoopPeakOutput(SLOT_INDEX, rkPeakOutput.get());
    rotationMotor.configPeakOutputForward(rkPeakOutput.get());
    rotationMotor.configPeakOutputReverse(-rkPeakOutput.get());
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

    inputs.pitchRadians = Units.degreesToRadians(pigeon.getPitch());
    inputs.rollRadians = Units.degreesToRadians(pigeon.getRoll());

    // check if we are stalled against a hard stop while retracting; if so, zero the encoder
    if (this.extensionSetpoint == 0
        && Math.abs(inputs.extensionVelocityMetersPerSec)
            < EXTENSION_MAX_STALL_VELOCITY_METERS_PER_SECOND) {
      stallCount++;
      if (stallCount > EXTENSION_MAX_STALL_DURATION_CYCLES) {
        extensionMotor.setSelectedSensorPosition(0);
      }
    } else {
      stallCount = 0;
    }

    // update tunables
    if (rkP.hasChanged() || rkI.hasChanged() || rkD.hasChanged() || rkPeakOutput.hasChanged()) {
      this.rotationMotor.config_kP(SLOT_INDEX, rkP.get());
      this.rotationMotor.config_kI(SLOT_INDEX, rkI.get());
      this.rotationMotor.config_kD(SLOT_INDEX, rkD.get());
      this.rotationMotor.configPeakOutputForward(rkPeakOutput.get());
      this.rotationMotor.configPeakOutputReverse(-rkPeakOutput.get());
      this.rotationMotor.configClosedLoopPeakOutput(SLOT_INDEX, rkPeakOutput.get());
    }

    if (ekP.hasChanged()
        || ekI.hasChanged()
        || ekD.hasChanged()
        || ekF.hasChanged()
        || ekPeakOutput.hasChanged()) {
      this.extensionMotor.config_kP(SLOT_INDEX, ekP.get());
      this.extensionMotor.config_kI(SLOT_INDEX, ekI.get());
      this.extensionMotor.config_kD(SLOT_INDEX, ekD.get());
      this.extensionMotor.config_kF(SLOT_INDEX, ekF.get());
      this.extensionMotor.configPeakOutputForward(ekPeakOutput.get());
      this.extensionMotor.configPeakOutputReverse(-ekPeakOutput.get());
      this.extensionMotor.configClosedLoopPeakOutput(SLOT_INDEX, ekPeakOutput.get());
    }

    if (extensionMagicMotionCruiseVelocity.hasChanged()
        || extensionMagicMotionAcceleration.hasChanged()
        || extensionMagicMotionSCurveStrength.hasChanged()) {
      this.extensionMotor.configMotionCruiseVelocity(
          mpsToFalconMotionMagicUnits(
              extensionMagicMotionCruiseVelocity.get(),
              EXTENSION_PULLEY_CIRCUMFERENCE,
              EXTENSION_GEAR_RATIO));
      this.extensionMotor.configMotionAcceleration(
          mpsToFalconMotionMagicUnits(
              extensionMagicMotionAcceleration.get(),
              EXTENSION_PULLEY_CIRCUMFERENCE,
              EXTENSION_GEAR_RATIO));
      this.extensionMotor.configMotionSCurveStrength(
          (int) (extensionMagicMotionSCurveStrength.get()));
    }

    // FIXME: if we pursue Motion Magic check the TunableNubmers to see if they have changed and
    // update the configuration
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
    this.extensionSetpoint = position;
    extensionMotor.set(
        TalonFXControlMode.MotionMagic,
        Conversions.metersToFalcon(position, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO),
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  public void setRotationPosition(double position, double arbitraryFeedForward) {
    rotationMotor.set(
        TalonFXControlMode.Position, // try MotionMagic later
        radiansToPigeon(position),
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  private static double mpsToFalconMotionMagicUnits(
      double mps, double circumference, double gearRatio) {
    double pulleyRotationsPerSecond = mps / circumference;
    double motorRotationsPerSecond = pulleyRotationsPerSecond * gearRatio;
    double ticksPerSecond = motorRotationsPerSecond * 2048.0;
    return ticksPerSecond / 10.0; // per 100 ms
  }
}
