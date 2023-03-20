package frc.robot.subsystems.elevator;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix.motion.TrajectoryPoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.lib.team3061.swerve.Conversions;
import frc.lib.team6328.util.TunableNumber;
import java.io.FileWriter;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  private final TunableNumber extensionKp =
      new TunableNumber("Elevator/ExtensionKp", SIM_EXTENSION_KP);
  private final TunableNumber extensionKi =
      new TunableNumber("Elevator/ExtensionKi", SIM_EXTENSION_KI);
  private final TunableNumber extensionKd =
      new TunableNumber("Elevator/ExtensionKd", SIM_EXTENSION_KD);
  private final TunableNumber rotationKp =
      new TunableNumber("Elevator/RotationKp", SIM_ROTATION_KP);
  private final TunableNumber rotationKi =
      new TunableNumber("Elevator/RotationKi", SIM_ROTATION_KI);
  private final TunableNumber rotationKd =
      new TunableNumber("Elevator/RotationKd", SIM_ROTATION_KD);

  private static final int LOOP_DT_MS = 10;

  /* Simulated Angle Motor PID Values */
  private static final double SIM_EXTENSION_KP = 2.0;
  private static final double SIM_EXTENSION_KI = 0.0;
  private static final double SIM_EXTENSION_KD = 0.0;

  /* Simulated Drive Motor PID Values */
  private static final double SIM_ROTATION_KP = 2.0;
  private static final double SIM_ROTATION_KI = 0.0;
  private static final double SIM_ROTATION_KD = 0.0;

  private double extensionSetpointMeters = 0.0;
  private double extensionAppliedVolts = 0.0;
  private double rotationAppliedVolts = 0.0;
  private double rotationSetpointRadians = 0.0;
  private boolean isExtensionOpenLoop = true;
  private boolean isRotationOpenLoop = true;

  // FIXME: check all values based on CAD
  private ElevatorSim elevatorSim =
      new ElevatorSim(DCMotor.getFalcon500(1), 3.0, 4.5, .05, 0.0, 1.651, false);
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          20.0,
          SingleJointedArmSim.estimateMOI(.9, 1.0),
          0.9,
          0.0,
          1.57,
          false);

  private PIDController extensionController =
      new PIDController(extensionKp.get(), extensionKi.get(), extensionKd.get());
  private PIDController rotationController =
      new PIDController(rotationKp.get(), rotationKi.get(), rotationKd.get());

  private Mechanism2d arm = new Mechanism2d(0.8636, 2.0);
  private MechanismRoot2d root = arm.getRoot("arm", 0.2, 0.1);
  private MechanismLigament2d elevator =
      root.append(new MechanismLigament2d("elevator", 0.6, 80.0));

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // update the models
    elevatorSim.update(LOOP_PERIOD_SECS);
    armSim.update(LOOP_PERIOD_SECS);

    // FIXME: the arm changes length; worth modeling?
    // FIXME: the force of gravity on the elevator changes; worth modeling?

    // update the inputs that will be logged
    inputs.extensionSetpointMeters = this.extensionSetpointMeters;
    inputs.extensionPositionMeters = elevatorSim.getPositionMeters();
    inputs.extensionVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.extensionClosedLoopErrorMeters = extensionController.getPositionError();
    inputs.extensionAppliedVolts = this.extensionAppliedVolts;
    inputs.extensionCurrentAmps = new double[] {Math.abs(elevatorSim.getCurrentDrawAmps())};
    inputs.extensionTempCelsius = new double[] {};

    inputs.rotationSetpointRadians = this.rotationSetpointRadians;
    inputs.rotationPositionRadians = armSim.getAngleRads();
    inputs.rotationVelocityRadiansPerSec = armSim.getVelocityRadPerSec();
    inputs.rotationClosedLoopErrorRadians = rotationController.getPositionError();
    inputs.rotationAppliedVolts = this.rotationAppliedVolts;
    inputs.rotationCurrentAmps = new double[] {Math.abs(armSim.getCurrentDrawAmps())};
    inputs.rotationTempCelsius = new double[] {};

    // update the Mechanism2d
    elevator.setAngle(inputs.rotationPositionRadians * 180.0 / Math.PI);
    elevator.setLength(0.3 + inputs.extensionPositionMeters);
    Logger.getInstance().recordOutput("Odometry/Mechanisms", this.arm);

    // update the tunable PID constants
    if (extensionKp.hasChanged() || extensionKi.hasChanged() || extensionKd.hasChanged()) {
      extensionController.setPID(extensionKp.get(), extensionKi.get(), extensionKd.get());
    }
    if (rotationKp.hasChanged() || rotationKi.hasChanged() || rotationKd.hasChanged()) {
      rotationController.setPID(rotationKp.get(), rotationKi.get(), rotationKd.get());
    }

    // calculate and apply the "on-board" controllers for the turn and drive motors
    if (!isExtensionOpenLoop) {
      this.extensionAppliedVolts =
          this.extensionController.calculate(
              inputs.extensionPositionMeters, this.extensionSetpointMeters);
      this.extensionAppliedVolts = MathUtil.clamp(this.extensionAppliedVolts, -12.0, 12.0);
      this.elevatorSim.setInputVoltage(this.extensionAppliedVolts);
    }

    if (!isRotationOpenLoop) {
      rotationAppliedVolts =
          rotationController.calculate(inputs.rotationPositionRadians, rotationSetpointRadians);
      rotationAppliedVolts = MathUtil.clamp(rotationAppliedVolts, -12.0, 12.0);
      armSim.setInputVoltage(rotationAppliedVolts);
    }
  }

  @Override
  public void setExtensionMotorPercentage(double percentage) {
    isExtensionOpenLoop = true;
    extensionController.reset();
    extensionAppliedVolts = MathUtil.clamp(percentage * 12.0, -12.0, 12.0);
    elevatorSim.setInputVoltage(extensionAppliedVolts);
  }

  @Override
  public void setRotationMotorPercentage(double percentage) {
    isRotationOpenLoop = true;
    rotationController.reset();
    rotationAppliedVolts = MathUtil.clamp(percentage * 12.0, -12.0, 12.0);
    armSim.setInputVoltage(rotationAppliedVolts);
  }

  @Override
  public void setPosition(
      double rotation,
      double rotationCruiseVelocity,
      double rotationAcceleration,
      double extension,
      double extensionCruiseVelocity,
      double extensionAcceleration,
      double rotationExtensionTimeOffset,
      boolean applyTimeOffsetAtStart) {
    try {
      isExtensionOpenLoop = false;
      isRotationOpenLoop = false;
      this.rotationSetpointRadians = rotation;
      this.extensionSetpointMeters = extension;

      Constraints rotationConstraints =
          new Constraints(
              radiansToPigeon(Units.degreesToRadians(rotationCruiseVelocity)),
              radiansToPigeon(Units.degreesToRadians(rotationAcceleration)));
      State rotationStartState = new State(radiansToPigeon(armSim.getAngleRads()), 0);
      TrapezoidProfile rotationProfile =
          new TrapezoidProfile(
              rotationConstraints, new State(radiansToPigeon(rotation), 0), rotationStartState);

      Constraints extensionConstraints =
          new Constraints(
              Conversions.metersToFalcon(
                  extensionCruiseVelocity, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO),
              Conversions.metersToFalcon(
                  extensionAcceleration, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO));
      State extensionStartState =
          new State(
              Conversions.metersToFalcon(
                  elevatorSim.getPositionMeters(),
                  EXTENSION_PULLEY_CIRCUMFERENCE,
                  EXTENSION_GEAR_RATIO),
              0);
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

      FileWriter trajectoryFile = new FileWriter("trajectory1.txt");
      trajectoryFile.write(
          "t\trotationPosition\trotationVelocity\textensionPosition\textensionVelocity\tx\ty\n");

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

        trajectoryFile.write(
            t
                + "\t"
                + Units.radiansToDegrees(pigeonToRadians(rotationPosition))
                + "\t"
                + Units.radiansToDegrees(pigeonToRadians(rotationVelocity))
                + "\t"
                + Units.metersToInches(
                    Conversions.falconToMeters(
                        extensionPosition, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO))
                + "\t"
                + Units.metersToInches(
                    Conversions.falconToMeters(
                        extensionVelocity, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO))
                + "\t"
                + (Units.metersToInches(
                            Conversions.falconToMeters(
                                extensionPosition,
                                EXTENSION_PULLEY_CIRCUMFERENCE,
                                EXTENSION_GEAR_RATIO))
                        * Math.cos(pigeonToRadians(rotationPosition))
                    - ((3.444 + 4.678)
                        / 2.0)) // subtract distance from average end of manipulator to front frame
                // perimeter
                + "\t"
                + (Units.metersToInches(
                            Conversions.falconToMeters(
                                extensionPosition,
                                EXTENSION_PULLEY_CIRCUMFERENCE,
                                EXTENSION_GEAR_RATIO))
                        * Math.sin(pigeonToRadians(rotationPosition))
                    + 17.3) // add distance from top of manipulator to ground
                + "\n");
      }

      trajectoryFile.close();
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }

  @Override
  public boolean isAtSetpoint() {
    boolean extensionIsAtSetpoint = false;
    boolean rotationIsAtSetpoint = false;

    double extensionPositionMeters = elevatorSim.getPositionMeters();
    double rotationPositionRadians = armSim.getAngleRads();

    if (this.rotationSetpointRadians == CONE_STORAGE_ROTATION_POSITION) {
      rotationIsAtSetpoint = rotationPositionRadians > this.rotationSetpointRadians;
    } else {
      rotationIsAtSetpoint =
          Math.abs(rotationPositionRadians - this.rotationSetpointRadians)
              < ELEVATOR_ROTATION_POSITION_TOLERANCE;
    }

    extensionIsAtSetpoint =
        Math.abs(extensionPositionMeters - this.extensionSetpointMeters)
            < ELEVATOR_EXTENSION_POSITION_TOLERANCE;

    return extensionIsAtSetpoint && rotationIsAtSetpoint;
  }

  private double pigeonToRadians(double counts) {
    return counts / PIGEON_UNITS_PER_ROTATION * (2 * Math.PI);
  }

  private double radiansToPigeon(double radians) {
    return radians / (2 * Math.PI) * PIGEON_UNITS_PER_ROTATION;
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
