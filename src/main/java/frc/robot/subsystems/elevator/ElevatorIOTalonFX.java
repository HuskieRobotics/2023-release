package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.Pigeon2;
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

    extensionConfig.SLOT0_KP = ekP.get();
    extensionConfig.SLOT0_KI = ekI.get();
    extensionConfig.SLOT0_KD = ekD.get();

    rotationConfig.SLOT0_KP = rkP.get();
    rotationConfig.SLOT0_KI = rkI.get();
    rotationConfig.SLOT0_KD = rkD.get();

    // FIXME: add remote filter to factory
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    talonFXConfig.remoteFilter0.remoteSensorDeviceID = PIGEON_ID;
    talonFXConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.Pigeon_Pitch;

    // FIXME: elevator specific TalonFX configure done here; including peak output as well

    extensionMotor = TalonFXFactory.createTalon(ELEVATOR_MOTOR_CAN_ID, canBusName, extensionConfig);
    rotationMotor =
        TalonFXFactory.createTalon(ROTATION_ELEVATOR_MOTOR_CAN_ID, canBusName, rotationConfig);

    rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

    // FIXME: make each of these TunableNumbers and use the TalonFXFactory configuration

    // FIXME: set current limits

    /* Motion Magic Configs */
    // rotationMotorConfig.motionAcceleration =
    //     ROTATION_ELEVATOR_ACCELERATION; // (distance units per 100 ms) per second
    // rotationMotorConfig.motionCruiseVelocity =
    //     ROTATION_MAX_ELEVATOR_VELOCITY; // distance units per 100 ms
    // rotationMotorConfig.motionCurveStrength = ROTATION_SCURVE_STRENGTH;

    // extensionMotorConfig.motionAcceleration =
    //     EXTENSION_ELEVATOR_ACCELERATION; // (distance units per 100 ms) per second
    // extensionMotorConfig.motionCruiseVelocity =
    //     EXTENSION_MAX_ELEVATOR_VELOCITY; // distance units per 100 ms
    // extensionMotorConfig.motionCurveStrength = EXTENSION_SCURVE_STRENGTH;

    /* Initialize */
    this.rotationMotor.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT_MS);

    // FIXME: initialie the extension sensor to a constant which represents the starting position of
    // the carriage when holding a cone

    // FIXME: configure the rotation Falcon to use the pigeon as an external sensor

    this.pigeon = new Pigeon2(PIGEON_ID);
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
    }

    if (ekP.hasChanged() || ekI.hasChanged() || ekD.hasChanged() || ekPeakOutput.hasChanged()) {
      this.extensionMotor.config_kP(SLOT_INDEX, rkP.get());
      this.extensionMotor.config_kI(SLOT_INDEX, rkI.get());
      this.extensionMotor.config_kD(SLOT_INDEX, rkD.get());
      this.extensionMotor.configPeakOutputForward(rkPeakOutput.get());
      this.extensionMotor.configPeakOutputReverse(rkPeakOutput.get());
    }
  }

  private double pigeonToRadians(double counts) {
    return counts / PIGEON_UNITS_PER_ROTATION * (2 * Math.PI);
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
