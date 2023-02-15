package frc.robot.subsystems.elevator;

import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.lib.team6328.util.TunableNumber;
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

  /* Simulated Angle Motor PID Values */
  private static final double SIM_EXTENSION_KP = 10.0;
  private static final double SIM_EXTENSION_KI = 0.0;
  private static final double SIM_EXTENSION_KD = 0.0;

  /* Simulated Drive Motor PID Values */
  private static final double SIM_ROTATION_KP = 10.0;
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
      new ElevatorSim(DCMotor.getFalcon500(1), 3.0, 4.5, .05, 0.0, 0.9, true);
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          20.0,
          SingleJointedArmSim.estimateMOI(.9, 1.0),
          0.9,
          0.0,
          1.57,
          10.0,
          true);

  // FIXME: add feedforward
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
    inputs.isControlEnabled = false;

    inputs.extensionSetpointMeters = this.extensionSetpointMeters;
    inputs.extensionPositionMeters = elevatorSim.getPositionMeters();
    inputs.extensionVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.extensionClosedLoopError = extensionController.getPositionError();
    inputs.extensionAppliedVolts = this.extensionAppliedVolts;
    inputs.extensionCurrentAmps = new double[] {Math.abs(elevatorSim.getCurrentDrawAmps())};
    inputs.extensionTempCelsius = new double[] {};

    inputs.rotationSetpointRadians = this.rotationSetpointRadians;
    inputs.rotationPositionRadians = armSim.getAngleRads();
    inputs.rotationVelocityRadiansPerSec = armSim.getVelocityRadPerSec();
    inputs.rotationClosedLoopError = rotationController.getPositionError();
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
  public void setExtensionPosition(double position, double arbitraryFeedForward) {
    isExtensionOpenLoop = false;
    extensionController.reset();
    extensionSetpointMeters = position;
  }

  @Override
  public void setRotationMotorPercentage(double percentage) {
    isRotationOpenLoop = true;
    rotationController.reset();
    rotationAppliedVolts = MathUtil.clamp(percentage * 12.0, -12.0, 12.0);
    armSim.setInputVoltage(rotationAppliedVolts);
  }

  @Override
  public void setRotationPosition(double position, double arbitraryFeedForward) {
    isRotationOpenLoop = false;
    rotationController.reset();
    rotationSetpointRadians = position;
  }
}
