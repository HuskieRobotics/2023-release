package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import frc.lib.team3061.util.CANDeviceFinder;
import frc.lib.team3061.util.CANDeviceId.CANDeviceType;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

// import frc.robot.util.CANDeviceFinder;
// import frc.robot.util.CANDeviceId.CANDeviceType;

public class ElevatorIOHardware implements ElevatorIO {
  private boolean isControlEnabled;

  private  WPI_TalonFX extensionMotor;
  private WPI_TalonFX rotationMotor;

  public void ElevatorIOTalonFX() {
    CANDeviceFinder can = new CANDeviceFinder();

    can.isDevicePresent(CANDeviceType.TALON, EXTENSION_MOTOR_CAN_ID, "Extension Motor");
    this.extensionMotor = new WPI_TalonFX(LEFT_ELEVATOR_MOTOR_CAN_ID);
    this.rotationMotor = new WPI_TalonFX(RIGHT_ELEVATOR_MOTOR_CAN_ID);

    // the following configuration is based on the CTRE example code

    /* Factory Default all hardware to prevent unexpected behaviour */
    this.rotationMotor.configFactoryDefault();
    this.extensionMotor.configFactoryDefault();

    /** Config Objects for motor controllers */
    TalonFXConfiguration rotationMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration extensionMotorConfig = new TalonFXConfiguration();

    /* Disable all motors */
    this.rotationMotor.set(TalonFXControlMode.PercentOutput, 0);
    this.extensionMotor.set(TalonFXControlMode.PercentOutput, 0);

    /* Set neutral modes */
    this.extensionMotor.setNeutralMode(NeutralMode.Brake);
    this.rotationMotor.setNeutralMode(NeutralMode.Brake);
    
    this.extensionMotor.follow(this.rotationMotor);

    /* Configure output */
    this.rotationMotor.setInverted(TalonFXInvertType.Clockwise);
    this.extensionMotor.setInverted(TalonFXInvertType.FollowMaster);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     *
     * this.extensionMotor.setSensorPhase(true)
     * this.rotationMotor.setSensorPhase(true)
     */

    /** Feedback Sensor Configuration */

    /** Distance Configs */

    /* Configure the left Talon's selected sensor as integrated sensor */
    extensionMotorConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    rotationMotorConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local
    // Feedback
    // Source

    // /* FPID for Distance */
    rotationMotorConfig.slot0.kF = ROTATION_POSITION_PID_F;
    rotationMotorConfig.slot0.kP = ROTATION_POSITION_PID_P;
    rotationMotorConfig.slot0.kI = ROTATION_POSITION_PID_I;
    rotationMotorConfig.slot0.kD = ROTATION_POSITION_PID_D;
    rotationMotorConfig.slot0.integralZone = ROTATION_POSITION_PID_I_ZONE;
    rotationMotorConfig.slot0.closedLoopPeakOutput = ROTATION_POSITION_PID_PEAK_OUTPUT;

    extensionMotorConfig.slot0.kF = EXTENSION_POSITION_PID_F;
    extensionMotorConfig.slot0.kP = EXTENSION_POSITION_PID_P;
    extensionMotorConfig.slot0.kI = EXTENSION_POSITION_PID_I;
    extensionMotorConfig.slot0.kD = EXTENSION_POSITION_PID_D;
    extensionMotorConfig.slot0.integralZone = EXTENSION_POSITION_PID_I_ZONE;
    extensionMotorConfig.slot0.closedLoopPeakOutput = EXTENSION_POSITION_PID_PEAK_OUTPUT;

    /* Config the neutral deadband. */
    rotationMotorConfig.neutralDeadband = 0.001;

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if sensor updates are
     * too slow - sensor deltas are very small per update, so derivative error never gets large
     * enough to be useful. - sensor movement is very slow causing the derivative error to be near
     * zero.
     */
    int closedLoopTimeMs = 1;
    rotationMotorConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    rotationMotorConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    rotationMotorConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    rotationMotorConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

    extensionMotorConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    extensionMotorConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    extensionMotorConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    extensionMotorConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

    /* Motion Magic Configs */
    rotationMotorConfig.motionAcceleration =
        ELEVATOR_ACCELERATION; // (distance units per 100 ms) per second
    rotationMotorConfig.motionCruiseVelocity = MAX_ELEVATOR_VELOCITY; // distance units per 100 ms
    rotationMotorConfig.motionCurveStrength = SCURVE_STRENGTH;

    extensionMotorConfig.motionAcceleration =
        ELEVATOR_ACCELERATION; // (distance units per 100 ms) per second
    extensionMotorConfig.motionCruiseVelocity = MAX_ELEVATOR_VELOCITY; // distance units per 100 ms
    extensionMotorConfig.motionCurveStrength = SCURVE_STRENGTH;


    /* APPLY the config settings */
    this.rotationMotor.configAllSettings(rotationMotorConfig);
    this.extensionMotor.configAllSettings(extensionMotorConfig);

    /* Initialize */
    this.rotationMotor.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT_MS);

    // these status frames aren't read; so, set these CAN frame periods to the maximum value
    // //  to reduce traffic on the bus
    this.rotationMotor.setStatusFramePeriod(
      StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.rotationMotor.setStatusFramePeriod(
      StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    this.extensionMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.extensionMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    this.pigeon = new Pigeon2(PIGEON_ID);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.isControlEnabled = isControlEnabled;

    inputs.extensionPosition = extensionMotor.getSelectedSensorPosition(SLOT_INDEX);
    inputs.extensionVelocity = extensionMotor.getSelectedSensorVelocity(SLOT_INDEX);
    inputs.extensionClosedLoopError = extensionMotor.getClosedLoopError(SLOT_INDEX);
    inputs.extensionAppliedVolts = extensionMotor.getMotorOutputVoltage();
    inputs.extensionCurrentAmps = new double[] {extensionMotor.getStatorCurrent()};
    inputs.extensionTempCelcius = new double[] {extensionMotor.getTemperature()};

    inputs.rotationPosition = rotationMotor.getSelectedSensorPosition(SLOT_INDEX);
    inputs.rotationVelocity = rotationMotor.getSelectedSensorVelocity(SLOT_INDEX);
    inputs.rotationClosedLoopError = rotationMotor.getClosedLoopError(SLOT_INDEX);
    inputs.rotationAppliedVolts = rotationMotor.getMotorOutputVoltage();
    inputs.rotationCurrentAmps = new double[] {rotationMotor.getStatorCurrent()};
    inputs.rotationTempCelcius = new double[] {rotationMotor.getTemperature()};

    inputs.pitch = pigeon.getPitch();
  }

  @Override
  public void setControlEnabled(boolean controlEnabled) {
    isControlEnabled = controlEnabled;
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
        TalonFXControlMode.Position,
        position,
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  public void setRotationPosition(double position, double arbitraryFeedForward) {
    rotationMotor.set(
        TalonFXControlMode.Position,
        position,
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  public void configureExtensionKF(double kF) {
    rotationMotor.config_kF(SLOT_INDEX, kF, TIMEOUT_MS);
  }

  @Override
  public void configureExtensionKP(double kP) {
    rotationMotor.config_kP(SLOT_INDEX, kP, TIMEOUT_MS);
  }

  @Override
  public void configureExtensionKI(double kI) {
    rotationMotor.config_kI(SLOT_INDEX, kI, TIMEOUT_MS);
  }

  @Override
  public void configureExtensionKD(double kD) {
    rotationMotor.config_kD(SLOT_INDEX, kD, TIMEOUT_MS);
  }

  @Override
  public void configExtensionClosedLoopPeakOutput(double peakOutput) {
    rotationMotor.configClosedLoopPeakOutput(SLOT_INDEX, peakOutput);
  }

  
  @Override
  public void configureRotationKF(double kF) {
    rotationMotor.config_kF(SLOT_INDEX, kF, TIMEOUT_MS);
  }

  @Override
  public void configureRotationKP(double kP) {
    rotationMotor.config_kP(SLOT_INDEX, kP, TIMEOUT_MS);
  }

  @Override
  public void configureRotationKI(double kI) {
    rotationMotor.config_kI(SLOT_INDEX, kI, TIMEOUT_MS);
  }

  @Override
  public void configureRotationKD(double kD) {
    rotationMotor.config_kD(SLOT_INDEX, kD, TIMEOUT_MS);
  }

  @Override
  public void configRotationClosedLoopPeakOutput(double peakOutput) {
    rotationMotor.configClosedLoopPeakOutput(SLOT_INDEX, peakOutput);
  }
}
