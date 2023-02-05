package frc.robot.subsystems.elevator;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.team6328.util.TunableNumber;

public class ElevatorIOSim implements ElevatorIO {
  private final TunableNumber extendKp = new TunableNumber("Elevator/ExtendKp", SIM_EXTEND_KP);
  private final TunableNumber extendKi = new TunableNumber("Elevator/ExtendKi", SIM_EXTEND_KI);
  private final TunableNumber extendKd = new TunableNumber("Elevator/ExtendKd", SIM_EXTEND_KD);
  private final TunableNumber rotateKp = new TunableNumber("Elevator/RotateKp", SIM_ROTATE_KP);
  private final TunableNumber rotateKi = new TunableNumber("Elevator/RotateKi", SIM_ROTATE_KI);
  private final TunableNumber rotateKd = new TunableNumber("Elevator/RotateKd", SIM_ROTATE_KD);

  /* Simulated Angle Motor PID Values */
  private static final double SIM_EXTEND_KP = 1.0;
  private static final double SIM_EXTEND_KI = 0.0;
  private static final double SIM_EXTEND_KD = 0.0;

  /* Simulated Drive Motor PID Values */
  private static final double SIM_ROTATE_KP = 1.0;
  private static final double SIM_ROTATE_KI = 0.0;
  private static final double SIM_ROTATE_KD = 0.0;

  private double extensionSetpointTicks = 0.0;
  private double extensionAppliedVolts = 0.0;
  private double rotationAppliedVolts = 0.0;
  private double rotationSetpointRadians = 0.0;

  // FIXME: check all values based on CAD
  private ElevatorSim elevatorSim =
      new ElevatorSim(DCMotor.getFalcon500(1), 3.0, 4.5, .05, 0.0, 0.9, true);
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          12.0,
          SingleJointedArmSim.estimateMOI(.9, 10.0),
          0.9,
          0.0,
          1.57,
          10.0,
          true);

  // FIXME: add feedforward
  private PIDController extensionController =
      new PIDController(extendKp.get(), extendKi.get(), extendKd.get());
  private PIDController rotationController =
      new PIDController(rotateKp.get(), rotateKi.get(), rotateKd.get());

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // update the models
    elevatorSim.update(LOOP_PERIOD_SECS);
    armSim.update(LOOP_PERIOD_SECS);

    // update the inputs that will be logged
    inputs.isControlEnabled = false;

    inputs.extensionSetpoint = this.extensionSetpointTicks;
    inputs.extensionPosition = elevatorSim.getPositionMeters();
    inputs.extensionVelocity = elevatorSim.getVelocityMetersPerSecond();
    inputs.extensionClosedLoopError = extensionController.getPositionError();
    inputs.extensionAppliedVolts = this.extensionAppliedVolts;
    inputs.extensionCurrentAmps = new double[] {Math.abs(elevatorSim.getCurrentDrawAmps())};
    inputs.extensionTempCelsius = new double[] {};

    inputs.rotationSetpoint = this.rotationSetpointRadians;
    inputs.rotationPosition = armSim.getAngleRads();
    inputs.rotationVelocity = armSim.getVelocityRadPerSec();
    inputs.rotationClosedLoopError = rotationController.getPositionError();
    inputs.rotationAppliedVolts = this.rotationAppliedVolts;
    inputs.rotationCurrentAmps = new double[] {Math.abs(armSim.getCurrentDrawAmps())};
    inputs.rotationTempCelsius = new double[] {};

    // update the tunable PID constants
    if (driveKp.hasChanged() || driveKi.hasChanged() || driveKd.hasChanged()) {
      driveController.setPID(driveKp.get(), driveKi.get(), driveKd.get());
    }
    if (turnKp.hasChanged() || turnKi.hasChanged() || turnKd.hasChanged()) {
      turnController.setPID(turnKp.get(), turnKi.get(), turnKd.get());
    }

    // calculate and apply the "on-board" controllers for the turn and drive motors
    turnAppliedVolts =
        turnController.calculate(turnRelativePositionRad, angleSetpointDeg * (Math.PI / 180.0));
    turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);

    if (!isDriveOpenLoop) {
      double velocityRadPerSec = driveSetpointMPS * (2.0 * Math.PI) / (MK4_L2_WHEEL_CIRCUMFERENCE);
      driveAppliedVolts =
          feedForward.calculate(velocityRadPerSec)
              + driveController.calculate(inputs.driveVelocityMetersPerSec, velocityRadPerSec);
      driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
      driveSim.setInputVoltage(driveAppliedVolts);
    }
  }
}
