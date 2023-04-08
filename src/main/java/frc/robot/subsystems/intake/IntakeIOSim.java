package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {

  /* Simulated Arm Motor PID Values */
  private static final double SIM_ROTATION_KP = 0.1;
  private static final double SIM_ROTATION_KI = 0.0;
  private static final double SIM_ROTATION_KD = 0.0;

  private final TunableNumber rotationKp = new TunableNumber("Intake/RotationKp", SIM_ROTATION_KP);
  private final TunableNumber rotationKi = new TunableNumber("Intake/RotationKi", SIM_ROTATION_KI);
  private final TunableNumber rotationKd = new TunableNumber("Intake/RotationKd", SIM_ROTATION_KD);

  // FIXME: delete after tuning
  private final TunableNumber armPosition = new TunableNumber("Intake/Position", -90.0);

  // FIXME: get the proper values from the CAD
  private static final double STALL_CURRENT_AMPS = 30.0;
  private static final double ROLLER_GEAR_RATIO = 1.0;
  private static final double ROLLER_MOMENT_OF_INERTIA = 0.1;

  private final SingleJointedArmSim armSim;
  private final FlywheelSim rollerSim;
  private double armAppliedVolts = 0.0;
  private double armPositionSetpointDeg = 0.0;
  private boolean isArmOpenLoop = true;
  private double rollerAppliedVolts = 0.0;

  private PIDController rotationController =
      new PIDController(rotationKp.get(), rotationKi.get(), rotationKd.get());

  private Mechanism2d robot = new Mechanism2d(0.8636, 2.0);
  private MechanismRoot2d root = robot.getRoot("intake", 0.7, 0.1);
  private MechanismLigament2d intake = root.append(new MechanismLigament2d("intake", 0.4, 0.0));

  public IntakeIOSim() {
    this.armSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            INTAKE_ROTATION_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(0.4, 1.0),
            0.25,
            0.0,
            Math.PI,
            true);
    this.rollerSim =
        new FlywheelSim(DCMotor.getNeo550(1), ROLLER_GEAR_RATIO, ROLLER_MOMENT_OF_INERTIA);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // update the models
    this.armSim.update(LOOP_PERIOD_SECS);
    this.rollerSim.update(LOOP_PERIOD_SECS);

    inputs.rotationPositionDeg = this.armSim.getAngleRads() * 180.0 / Math.PI;
    inputs.rotationVelocityDegPerSec = this.armSim.getVelocityRadPerSec() * 180.0 / Math.PI;
    inputs.rotationClosedLoopError = rotationController.getPositionError();
    inputs.rotationAppliedPercentage = armAppliedVolts / 12.0;
    inputs.rotationStatorCurrentAmps = new double[] {this.armSim.getCurrentDrawAmps()};
    inputs.rotationTempCelsius = new double[] {};

    inputs.rollerAppliedPercentage = rollerAppliedVolts / 12.0;
    inputs.rollerTempCelsius = new double[] {0.0};

    // update the Mechanism2d
    intake.setAngle(180.0 - inputs.rotationVelocityDegPerSec); // adjust for model
    Logger.getInstance().recordOutput("Odometry/Mechanisms", this.robot);

    // update the tunable PID constants
    if (rotationKp.hasChanged() || rotationKi.hasChanged() || rotationKd.hasChanged()) {
      rotationController.setPID(rotationKp.get(), rotationKi.get(), rotationKd.get());
    }

    if (armPosition.hasChanged()) {
      this.setRotationPosition(armPosition.get(), 0.0);
    }

    // calculate and apply the "on-board" controllers for the turn and drive motors
    if (!isArmOpenLoop) {
      armAppliedVolts =
          rotationController.calculate(
              inputs.rotationVelocityDegPerSec, this.armPositionSetpointDeg);
      armAppliedVolts = MathUtil.clamp(armAppliedVolts, -12.0, 12.0);
      armSim.setInputVoltage(armAppliedVolts);
    }
  }

  @Override
  public void setRotationMotorPercentage(double percentage) {
    this.isArmOpenLoop = true;
    this.armAppliedVolts = percentage * 12.0;
    this.armSim.setInputVoltage(this.armAppliedVolts);
  }

  @Override
  public void setRotationPosition(double position, double arbitraryFeedForward) {
    this.isArmOpenLoop = false;
    this.rotationController.reset();
    this.armPositionSetpointDeg = position;
  }

  @Override
  public void setRollerMotorPercentage(double percentage) {
    this.rollerAppliedVolts = percentage * 12.0;
    this.rollerSim.setInputVoltage(this.rollerAppliedVolts);
  }
}
