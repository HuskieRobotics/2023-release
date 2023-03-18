package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.Conversions;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX extensionMotor;
  private TalonFX rotationMotor;
  private final String canBusName = RobotConfig.getInstance().getCANBusName();
  private Pigeon2 pigeon;
  private int stallCount;
  private double previousRotationPosition = -1.0;
  private int rotationStuckCount;
  private double extensionSetpoint = -1.0;
  private double rotationSetpoint = 0.0;
  private BufferedTrajectoryPointStream rotationBufferedStream =
      new BufferedTrajectoryPointStream();
  private BufferedTrajectoryPointStream extensionBufferedStream =
      new BufferedTrajectoryPointStream();

  private static final int LOOP_DT_MS = 10;

  private final TunableNumber rkP =
      new TunableNumber("ElevatorRotation/kP", ROTATION_POSITION_PID_P);
  private final TunableNumber rkI =
      new TunableNumber("ElevatorRotation/kI", ROTATION_POSITION_PID_I);
  private final TunableNumber rkD =
      new TunableNumber("ElevatorRotation/kD", ROTATION_POSITION_PID_D);
  private final TunableNumber rkF =
      new TunableNumber("ElevatorRotation/kF", ROTATION_POSITION_PID_F);
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

  private final TunableNumber rotationMotionProfileAcceleration =
      new TunableNumber(
          "ElevatorRotation/acceleration(degpsps)",
          ROTATION_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND);
  private final TunableNumber rotationMotionProfileExtensionCruiseVelocity =
      new TunableNumber(
          "ElevatorRotation/maxVelocity(degps)", MAX_ROTATION_VELOCITY_DEGREES_PER_SECOND);

  private final TunableNumber extensionMotionProfileAcceleration =
      new TunableNumber(
          "ElevatorExtension/acceleration(mpsps)",
          EXTENSION_ACCELERATION_METERS_PER_SECOND_PER_SECOND);
  private final TunableNumber extensionMotionProfileExtensionCruiseVelocity =
      new TunableNumber(
          "ElevatorExtension/maxExtensionVelocity(mps)", MAX_EXTENSION_VELOCITY_METERS_PER_SECOND);
  private final TunableNumber extensionMotionProfileRetractionCruiseVelocity =
      new TunableNumber(
          "ElevatorExtension/maxRetractionVelocity(mps)",
          MAX_RETRACTION_VELOCITY_METERS_PER_SECOND);

  private final TunableNumber rotationStuckMinPositionDelta =
      new TunableNumber("ElevatorRotation/StuckMinPositionDelta", 0.005);
  private final TunableNumber rotationStuckCycles =
      new TunableNumber("ElevatorRotation/StuckCycles", 5);

  private final TunableNumber rotationStorageHoldCurrent =
      new TunableNumber("ElevatorRotation/StorageHoldCurrent", 5);

  public ElevatorIOTalonFX() {
    // create and configure the Pigeon
    this.pigeon = new Pigeon2(PIGEON_ID, RobotConfig.getInstance().getCANBusName());
    Pigeon2Configuration config = new Pigeon2Configuration();

    // set mount pose as rolled 90 degrees clockwise
    config.MountPoseYaw = 0;
    config.MountPoseRoll = -90.0;
    this.pigeon.configAllSettings(config);
    this.pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 9);

    // TalonFXFactory.Configuration extensionConfig = new TalonFXFactory.Configuration();
    // TalonFXFactory.Configuration rotationConfig = new TalonFXFactory.Configuration();

    // extensionConfig.INVERTED = EXTENSION_INVERTED;
    // extensionConfig.NEUTRAL_MODE = NeutralMode.Brake;
    // extensionConfig.SLOT0_KP = ekP.get();
    // extensionConfig.SLOT0_KI = ekI.get();
    // extensionConfig.SLOT0_KD = ekD.get();
    // extensionConfig.SLOT0_KF = ekF.get();

    // rotationConfig.INVERTED = ROTATION_INVERTED;
    // rotationConfig.NEUTRAL_MODE = NeutralMode.Brake;
    // rotationConfig.SLOT0_KP = rkP.get();
    // rotationConfig.SLOT0_KI = rkI.get();
    // rotationConfig.SLOT0_KD = rkD.get();
    // rotationConfig.SLOT0_KF = rkF.get();

    // extensionConfig.STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 40, 50, 1);
    // rotationConfig.STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 30, 40, 1);

    // rotationConfig.REMOTE_SENSOR_DEVICE_ID = PIGEON_ID;
    // rotationConfig.REMOTE_SENSOR_SOURCE = RemoteSensorSource.Pigeon_Pitch;
    // rotationConfig.SENSOR_PHASE = true;

    // extensionConfig.NEUTRAL_DEADBAND = 0.001;
    // rotationConfig.NEUTRAL_DEADBAND = 0.001;

    // extensionConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 9;
    // extensionConfig.MOTION_MAGIC_STATUS_FRAME_RATE_MS = 9;
    // extensionConfig.BASE_PIDF0_STATUS_FRAME_RATE_MS = 9;
    // extensionConfig.FEEDBACK_INTEGRATED_STATUS_FRAME_RATE_MS = 9;
    // extensionConfig.BRUSHLESS_CURRENT_STATUS_FRAME_RATE_MS = 19;

    // rotationConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 9;
    // rotationConfig.MOTION_MAGIC_STATUS_FRAME_RATE_MS = 9;
    // rotationConfig.BASE_PIDF0_STATUS_FRAME_RATE_MS = 9;
    // rotationConfig.FEEDBACK_INTEGRATED_STATUS_FRAME_RATE_MS = 9;
    // rotationConfig.BRUSHLESS_CURRENT_STATUS_FRAME_RATE_MS = 19;

    // extensionMotor =
    //     TalonFXFactory.createTalon(EXTENSION_ELEVATOR_MOTOR_CAN_ID, canBusName, extensionConfig);
    // rotationMotor =
    //     TalonFXFactory.createTalon(ROTATION_ELEVATOR_MOTOR_CAN_ID, canBusName, rotationConfig);

    // rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    // rotationMotor.config_IntegralZone(0, radiansToPigeon(ROTATION_POSITION_PID_I_ZONE));

    // extensionMotor.setSelectedSensorPosition(
    //     Conversions.metersToFalcon(
    //         Units.inchesToMeters(START_EXTENSION_POSITION_INCHES),
    //         EXTENSION_PULLEY_CIRCUMFERENCE,
    //         EXTENSION_GEAR_RATIO));

    // extensionMotor.configClosedLoopPeakOutput(SLOT_INDEX, ekPeakOutput.get());
    // extensionMotor.configPeakOutputForward(ekPeakOutput.get());
    // extensionMotor.configPeakOutputReverse(-ekPeakOutput.get());

    // rotationMotor.configClosedLoopPeakOutput(SLOT_INDEX, rkPeakOutput.get());
    // rotationMotor.configPeakOutputForward(rkPeakOutput.get());
    // rotationMotor.configPeakOutputReverse(-rkPeakOutput.get());

    rotationMotor = new WPI_TalonFX(ROTATION_ELEVATOR_MOTOR_CAN_ID, "canbus1");
    extensionMotor = new WPI_TalonFX(EXTENSION_ELEVATOR_MOTOR_CAN_ID, "canbus1");

    TalonFXConfiguration rotationConfig = new TalonFXConfiguration(); // factory default settings
    TalonFXConfiguration extensionConfig = new TalonFXConfiguration(); // factory default settings
    rotationConfig.neutralDeadband = 0.001; /* 0.1 % super small for best low-speed control */
    rotationConfig.slot0.kF = rkF.get();
    rotationConfig.slot0.kP = rkP.get();
    rotationConfig.slot0.kI = rkI.get();
    rotationConfig.slot0.kD = rkD.get();
    rotationConfig.slot0.integralZone = (int) 400;
    rotationConfig.slot0.closedLoopPeakOutput = rkPeakOutput.get();

    rotationConfig.remoteFilter0.remoteSensorDeviceID = PIGEON_ID;
    rotationConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.Pigeon_Pitch;

    // rotationConfig.slot0.allowableClosedloopError // left default for this
    // example
    // rotationConfig.slot0.maxIntegralAccumulator; // left default for this example
    // rotationConfig.slot0.closedLoopPeriod; // left default for this example
    rotationMotor.configAllSettings(rotationConfig);

    rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    rotationMotor.setSensorPhase(true);
    rotationMotor.setNeutralMode(NeutralMode.Brake);
    rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 9);

    /* pick the sensor phase and desired direction */
    rotationMotor.setInverted(TalonFXInvertType.CounterClockwise);

    /* _config the master specific settings */
    extensionConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    extensionConfig.neutralDeadband = 0.001; /* 0.1 % super small for best low-speed control */
    extensionConfig.slot0.kF = ekF.get();
    extensionConfig.slot0.kP = ekP.get();
    extensionConfig.slot0.kI = ekI.get();
    extensionConfig.slot0.kD = ekD.get();
    extensionConfig.slot0.integralZone = (int) 400;
    extensionConfig.slot0.closedLoopPeakOutput = ekPeakOutput.get();
    // extensionConfig.slot0.allowableClosedloopError // left default for this
    // example
    // extensionConfig.slot0.maxIntegralAccumulator; // left default for this
    // example
    // extensionConfig.slot0.closedLoopPeriod; // left default for this example
    extensionMotor.configAllSettings(extensionConfig);

    /* pick the sensor phase and desired direction */
    extensionMotor.setInverted(TalonFXInvertType.CounterClockwise);
    extensionMotor.setNeutralMode(NeutralMode.Brake);
    extensionMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 9);
    extensionMotor.setSelectedSensorPosition(0.0);
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

    logMotionProfileStatus();

    // check if we are stalled against a hard stop while retracting; if so, zero the encoder
    if (this.extensionSetpoint == 0
        && inputs.extensionPositionMeters
            < CONE_STORAGE_EXTENSION_POSITION + STORAGE_EXTENSION_POSITION_TOLERANCE
        && Math.abs(inputs.extensionVelocityMetersPerSec)
            < EXTENSION_MAX_STALL_VELOCITY_METERS_PER_SECOND) {
      stallCount++;
      if (stallCount > EXTENSION_MAX_STALL_DURATION_CYCLES) {
        extensionMotor.setSelectedSensorPosition(0);
        setExtensionMotorPercentage(0.0);
        Logger.getInstance().recordOutput("Elevator/extensionReZero", true);
      }
    } else {
      stallCount = 0;
      Logger.getInstance().recordOutput("Elevator/extensionReZero", false);
    }

    // stall the rotation motor against the mechanical hard stop when moving into the storage
    // position
    if (this.rotationSetpoint == CONE_STORAGE_ROTATION_POSITION
        && inputs.rotationPositionRadians
            > CONE_STORAGE_ROTATION_POSITION - STORAGE_ROTATION_POSITION_TOLERANCE) {
      setRotationMotorCurrent(rotationStorageHoldCurrent.get());
    }

    // check if the elevator is stuck while trying to rotate; if so, stop the rotation
    // if (Math.abs(previousRotationPosition - inputs.rotationPositionRadians)
    //     < rotationStuckMinPositionDelta.get()) {
    //   rotationStuckCount++;
    //   if (rotationStuckCount > rotationStuckCycles.get()) {
    //     setRotationMotorPercentage(0.0);
    //     Logger.getInstance().recordOutput("Elevator/rotationStuck", true);
    //   }
    // } else {
    //   rotationStuckCount = 0;
    //   Logger.getInstance().recordOutput("Elevator/rotationStuck", false);
    // }
    // this.previousRotationPosition = inputs.rotationPositionRadians;

    // update tunables
    if (rkP.hasChanged()
        || rkI.hasChanged()
        || rkD.hasChanged()
        || rkF.hasChanged()
        || rkPeakOutput.hasChanged()) {
      this.rotationMotor.config_kP(SLOT_INDEX, rkP.get());
      this.rotationMotor.config_kI(SLOT_INDEX, rkI.get());
      this.rotationMotor.config_kD(SLOT_INDEX, rkD.get());
      this.rotationMotor.config_kF(SLOT_INDEX, rkF.get());
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
  public void setPosition(
      double rotation,
      double extension,
      double rotationExtensionTimeOffset,
      boolean applyTimeOffsetAtStart) {

    this.rotationSetpoint = rotation;
    this.extensionSetpoint = extension;

    // this may allow the current control mode to stop and switch to motion profiling
    if (this.rotationMotor.getControlMode() == ControlMode.Current) {
      this.rotationMotor.set(ControlMode.PercentOutput, 0.0);
    }

    // use different motion profiles for the extension based on direction
    double extensionCruiseVelocity;
    if (Conversions.metersToFalcon(
            this.extensionSetpoint, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO)
        > this.extensionMotor.getSelectedSensorPosition(SLOT_INDEX)) {
      extensionCruiseVelocity = extensionMotionProfileExtensionCruiseVelocity.get();
    } else {
      extensionCruiseVelocity = extensionMotionProfileRetractionCruiseVelocity.get();
    }

    Constraints rotationConstraints =
        new Constraints(
            radiansToPigeon(
                Units.degreesToRadians(rotationMotionProfileExtensionCruiseVelocity.get())),
            radiansToPigeon(Units.degreesToRadians(rotationMotionProfileAcceleration.get())));
    State rotationStartState =
        new State(this.rotationMotor.getSelectedSensorPosition(SLOT_INDEX), 0);
    TrapezoidProfile rotationProfile =
        new TrapezoidProfile(
            rotationConstraints, new State(radiansToPigeon(rotation), 0), rotationStartState);

    Constraints extensionConstraints =
        new Constraints(
            Conversions.metersToFalcon(
                extensionCruiseVelocity, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO),
            Conversions.metersToFalcon(
                extensionMotionProfileAcceleration.get(),
                EXTENSION_PULLEY_CIRCUMFERENCE,
                EXTENSION_GEAR_RATIO));
    State extensionStartState =
        new State(this.extensionMotor.getSelectedSensorPosition(SLOT_INDEX), 0);
    TrapezoidProfile extensionProfile =
        new TrapezoidProfile(
            extensionConstraints,
            new State(
                Conversions.metersToFalcon(
                    extension, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO),
                0),
            extensionStartState);

    // subtract durationDifference to the time when generating the extension profile
    double durationDifference =
        rotationProfile.totalTime() - extensionProfile.totalTime() - rotationExtensionTimeOffset;

    double extensionTimeOffset = 0.0;
    double rotationTimeOffset = 0.0;

    // FIXME: explain this algorithm
    if (applyTimeOffsetAtStart) {
      if (rotationExtensionTimeOffset > 0) {
        extensionTimeOffset = -rotationExtensionTimeOffset;
      } else {
        rotationTimeOffset = rotationExtensionTimeOffset;
      }

    } else {
      if (durationDifference > 0) {
        extensionTimeOffset = -durationDifference;
      } else {
        rotationTimeOffset = durationDifference;
      }
    }

    TrajectoryPoint point = new TrajectoryPoint();

    /* clear the buffer, in case it was used elsewhere */
    rotationBufferedStream.Clear();
    extensionBufferedStream.Clear();

    for (double t = 0;
        !rotationProfile.isFinished(t + rotationTimeOffset - LOOP_DT_MS / 1000.0)
            || !extensionProfile.isFinished(t + extensionTimeOffset - LOOP_DT_MS / 1000.0);
        t += LOOP_DT_MS / 1000.0) {

      boolean lastPoint =
          rotationProfile.isFinished(t + rotationTimeOffset)
              && extensionProfile.isFinished(t + extensionTimeOffset);

      double rotationPosition;
      double rotationVelocity;
      double extensionPosition;
      double extensionVelocity;

      // we may invoke calculate after the end of the profile; if we do, it just
      // returns the goal state
      if (t + rotationTimeOffset >= 0) {
        rotationPosition = rotationProfile.calculate(t + rotationTimeOffset).position;
        rotationVelocity = rotationProfile.calculate(t + rotationTimeOffset).velocity;
      } else {
        rotationPosition = rotationStartState.position;
        rotationVelocity = rotationStartState.velocity;
      }

      if (t + extensionTimeOffset >= 0) {
        extensionPosition = extensionProfile.calculate(t + extensionTimeOffset).position;
        extensionVelocity = extensionProfile.calculate(t + extensionTimeOffset).velocity;
      } else {
        extensionPosition = extensionStartState.position;
        extensionVelocity = extensionStartState.velocity;
      }

      point.timeDur = LOOP_DT_MS;
      point.position = rotationPosition;
      point.velocity = rotationVelocity / 10;
      point.auxiliaryPos = 0;
      point.auxiliaryVel = 0;
      point.profileSlotSelect0 = SLOT_INDEX; /* which set of gains would you like to use [0,3]? */
      point.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
      point.zeroPos = false;
      point.isLastPoint = lastPoint; /* set this to true on the last point */
      point.arbFeedFwd =
          calculateRotationFeedForward(
              Units.metersToInches(
                  Conversions.falconToMeters(
                      extensionPosition, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO)),
              pigeonToRadians(rotationPosition));

      rotationBufferedStream.Write(point);

      point.timeDur = LOOP_DT_MS;
      point.position = extensionPosition;
      point.velocity = extensionVelocity / 10;
      point.auxiliaryPos = 0;
      point.auxiliaryVel = 0;
      point.profileSlotSelect0 = SLOT_INDEX; /* which set of gains would you like to use [0,3]? */
      point.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
      point.zeroPos = false;
      point.isLastPoint = lastPoint; /* set this to true on the last point */
      point.arbFeedFwd =
          calculateExtensionFeedForward(
              Units.metersToInches(
                  Conversions.falconToMeters(
                      extensionPosition, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO)),
              pigeonToRadians(rotationPosition));

      extensionBufferedStream.Write(point);
    }

    rotationMotor.startMotionProfile(
        rotationBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
    extensionMotor.startMotionProfile(
        extensionBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
  }

  @Override
  public boolean isAtSetpoint() {
    boolean extensionIsAtSetpoint = false;
    boolean rotationIsAtSetpoint = false;

    double extensionPositionMeters =
        Conversions.falconToMeters(
            extensionMotor.getSelectedSensorPosition(SLOT_INDEX),
            EXTENSION_PULLEY_CIRCUMFERENCE,
            EXTENSION_GEAR_RATIO);
    double rotationPositionRadians =
        pigeonToRadians(rotationMotor.getSelectedSensorPosition(SLOT_INDEX));

    if (this.rotationSetpoint == CONE_STORAGE_ROTATION_POSITION) {
      rotationIsAtSetpoint = rotationPositionRadians > this.rotationSetpoint;
    } else {
      rotationIsAtSetpoint =
          Math.abs(rotationPositionRadians - this.rotationSetpoint)
              < ELEVATOR_ROTATION_POSITION_TOLERANCE;
    }

    extensionIsAtSetpoint =
        Math.abs(extensionPositionMeters - this.extensionSetpoint)
            < ELEVATOR_EXTENSION_POSITION_TOLERANCE;

    // return extensionIsAtSetpoint && rotationIsAtSetpoint;
    return rotationMotor.isMotionProfileFinished() && extensionMotor.isMotionProfileFinished();
  }

  @Override
  public void autoZeroExtension() {
    this.setExtensionMotorPercentage(-MAX_MANUAL_POWER_ROTATION);
    this.extensionSetpoint = 0.0;
  }

  private void setRotationMotorCurrent(double current) {
    rotationMotor.set(ControlMode.Current, current);
  }

  private double pigeonToRadians(double counts) {
    return counts / PIGEON_UNITS_PER_ROTATION * (2 * Math.PI);
  }

  private double radiansToPigeon(double radians) {
    return radians / (2 * Math.PI) * PIGEON_UNITS_PER_ROTATION;
  }

  private void logMotionProfileStatus() {
    MotionProfileStatus status = new MotionProfileStatus();
    this.rotationMotor.getMotionProfileStatus(status);

    Logger.getInstance().recordOutput("Elevator/rotTopBufferRem", status.topBufferRem);
    Logger.getInstance().recordOutput("Elevator/rotTopBufferCnt", status.topBufferCnt);
    Logger.getInstance().recordOutput("Elevator/rotBtmBufferCnt", status.btmBufferCnt);
    Logger.getInstance().recordOutput("Elevator/rotHasUnderrun", status.hasUnderrun);
    Logger.getInstance().recordOutput("Elevator/rotIsUnderrun", status.isUnderrun);
    Logger.getInstance().recordOutput("Elevator/rotActivePointValid", status.activePointValid);
    Logger.getInstance().recordOutput("Elevator/rotIsLast", status.isLast);
    Logger.getInstance().recordOutput("Elevator/rotOutputEnable", status.outputEnable.toString());
    Logger.getInstance().recordOutput("Elevator/rotTimeDurMs", status.timeDurMs);

    this.extensionMotor.getMotionProfileStatus(status);

    Logger.getInstance().recordOutput("Elevator/extTopBufferRem", status.topBufferRem);
    Logger.getInstance().recordOutput("Elevator/extTopBufferCnt", status.topBufferCnt);
    Logger.getInstance().recordOutput("Elevator/extBtmBufferCnt", status.btmBufferCnt);
    Logger.getInstance().recordOutput("Elevator/extHasUnderrun", status.hasUnderrun);
    Logger.getInstance().recordOutput("Elevator/extIsUnderrun", status.isUnderrun);
    Logger.getInstance().recordOutput("Elevator/extActivePointValid", status.activePointValid);
    Logger.getInstance().recordOutput("Elevator/extIsLast", status.isLast);
    Logger.getInstance().recordOutput("Elevator/extOutputEnable", status.outputEnable.toString());
    Logger.getInstance().recordOutput("Elevator/extTimeDurMs", status.timeDurMs);
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
  private static final double F_COLLAPSED_ELEVATOR_AT_11_DEG = 25.712; // FIXME: update after tuning
  private static final double MIN_MOTOR_POWER_TO_EXTEND_CARRIAGE_AT_60_DEG = 0.05; // FIXME: tune
  private static final double MIN_MOTOR_POWER_TO_ROTATE_COLLAPSED_ELEVATOR_AT_11_DEG =
      0.05; // FIXME: tune

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

    // FIXME: delete after tuning
    Logger.getInstance().recordOutput("Elevator/rotationFeedForwardF3", F3);

    double feedForward =
        (MIN_MOTOR_POWER_TO_ROTATE_COLLAPSED_ELEVATOR_AT_11_DEG / F_COLLAPSED_ELEVATOR_AT_11_DEG)
            * F3;
    Logger.getInstance().recordOutput("Elevator/rotationFeedForward", feedForward);
    return feedForward;
  }

  private static double calculateExtensionFeedForward(double extension, double rotation) {
    double mass;
    if (extension <= MAX_EXTENSION_BEFORE_MOVING_STAGE_ENGAGEMENT) {
      mass = CARRIAGE_MASS;
    } else {
      mass = (CARRIAGE_MASS + MOVING_STAGE_MASS) / 2.0; // two belts are now in tension
    }

    double f = mass * Math.sin(rotation);

    double feedForward =
        (MIN_MOTOR_POWER_TO_EXTEND_CARRIAGE_AT_60_DEG / (CARRIAGE_MASS * 0.866)) * f;
    Logger.getInstance().recordOutput("Elevator/extensionFeedForward", feedForward);
    return feedForward;
  }
}
