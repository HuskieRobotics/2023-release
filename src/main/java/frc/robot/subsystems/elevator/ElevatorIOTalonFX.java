package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.team254.drivers.TalonFXFactory;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.CANDeviceFinder;
import frc.lib.team6328.util.TunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX extensionMotor;
  private TalonFX rotationMotor;
  private final String canBusName = RobotConfig.getInstance().getCANBusName();
  private Pigeon2 pigeon;

  public ElevatorIOTalonFX() {
    CANDeviceFinder can = new CANDeviceFinder();

    extensionMotor = TalonFXFactory.createDefaultTalon(0, canBusName);
    rotationMotor = TalonFXFactory.createDefaultTalon(1, canBusName);

    // the following configuration is based on the CTRE example code

    /* Factory Default all hardware to prevent unexpected behavior */
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

    // /* PID for Distance */

    final TunableNumber rkF = new TunableNumber("ElevatorRotation/kF", ROTATION_POSITION_PID_F);
    final TunableNumber rkP = new TunableNumber("ElevatorRotation/kP", ROTATION_POSITION_PID_P);
    final TunableNumber rkI = new TunableNumber("ElevatorRotation/kI", ROTATION_POSITION_PID_I);
    final TunableNumber rkD = new TunableNumber("ElevatorRotation/kD", ROTATION_POSITION_PID_D);
    final TunableNumber rkIz =
        new TunableNumber("ElevatorRotation/kIz", ROTATION_POSITION_PID_I_ZONE);
    final TunableNumber rkPeakOutput =
        new TunableNumber("ElevatorRotation/kPeakOutput", ROTATION_POSITION_PID_PEAK_OUTPUT);

    final TunableNumber ekF = new TunableNumber("ElevatorExtension/kF", EXTENSION_POSITION_PID_F);
    final TunableNumber ekP = new TunableNumber("ElevatorExtension/kP", EXTENSION_POSITION_PID_P);
    final TunableNumber ekI = new TunableNumber("ElevatorExtension/kI", EXTENSION_POSITION_PID_I);
    final TunableNumber ekD = new TunableNumber("ElevatorExtension/kD", EXTENSION_POSITION_PID_D);
    final TunableNumber ekIz =
        new TunableNumber("ElevatorExtension/kIz", EXTENSION_POSITION_PID_I_ZONE);
    final TunableNumber ekPeakOutput =
        new TunableNumber("ElevatorExtension/kPeakOutput", EXTENSION_POSITION_PID_PEAK_OUTPUT);

    final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(ekF.get(), ekP.get(), ekD.get());
    final PIDController extensionController = new PIDController(ekP.get(), ekI.get(), ekD.get());
    final PIDController rotationController = new PIDController(rkP.get(), rkI.get(), rkD.get());

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
        ROTATION_ELEVATOR_ACCELERATION; // (distance units per 100 ms) per second
    rotationMotorConfig.motionCruiseVelocity =
        ROTATION_MAX_ELEVATOR_VELOCITY; // distance units per 100 ms
    rotationMotorConfig.motionCurveStrength = ROTATION_SCURVE_STRENGTH;

    extensionMotorConfig.motionAcceleration =
        EXTENSION_ELEVATOR_ACCELERATION; // (distance units per 100 ms) per second
    extensionMotorConfig.motionCruiseVelocity =
        EXTENSION_MAX_ELEVATOR_VELOCITY; // distance units per 100 ms
    extensionMotorConfig.motionCurveStrength = EXTENSION_SCURVE_STRENGTH;

    /* APPLY the config settings */
    this.rotationMotor.configAllSettings(rotationMotorConfig);
    this.extensionMotor.configAllSettings(extensionMotorConfig);

    /* Initialize */
    this.rotationMotor.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT_MS);

    // these status frames aren't read; so, set these CAN frame periods to the maximum value
    // //  to reduce traffic on the bus
    this.rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.rotationMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    this.extensionMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.extensionMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    this.pigeon = new Pigeon2(PIGEON_ID);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.extensionPositionMeters = extensionMotor.getSelectedSensorPosition(SLOT_INDEX);
    inputs.extensionVelocityMetersPerSec = extensionMotor.getSelectedSensorVelocity(SLOT_INDEX);
    inputs.extensionClosedLoopError = extensionMotor.getClosedLoopError(SLOT_INDEX);
    inputs.extensionAppliedVolts = extensionMotor.getMotorOutputVoltage();
    inputs.extensionCurrentAmps = new double[] {extensionMotor.getStatorCurrent()};
    inputs.extensionTempCelsius = new double[] {extensionMotor.getTemperature()};

    inputs.rotationPositionRadians = rotationMotor.getSelectedSensorPosition(SLOT_INDEX);
    inputs.rotationVelocityRadiansPerSec = rotationMotor.getSelectedSensorVelocity(SLOT_INDEX);
    inputs.rotationClosedLoopError = rotationMotor.getClosedLoopError(SLOT_INDEX);
    inputs.rotationAppliedVolts = rotationMotor.getMotorOutputVoltage();
    inputs.rotationCurrentAmps = new double[] {rotationMotor.getStatorCurrent()};
    inputs.rotationTempCelsius = new double[] {rotationMotor.getTemperature()};

    inputs.pitch = pigeon.getPitch();
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
