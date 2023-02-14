package frc.robot.subsystems.manipulator;

import static frc.robot.Constants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ManipulatorIOSim implements ManipulatorIO {

  private static final double GEAR_RATIO = 54.0;
  private static final double STALL_CURRENT_AMPS = 30.0;

  private final DigitalInput manipulatorSensor;
  private final SingleJointedArmSim falconSim;
  private double appliedVolts = 0.0;

  public ManipulatorIOSim() {
    this.manipulatorSensor = new DigitalInput(9);
    this.falconSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(0.25, 1.0),
            0.25,
            0.0,
            0.52,
            false);
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    // update the models
    falconSim.update(LOOP_PERIOD_SECS);

    inputs.positionDeg = falconSim.getAngleRads() * 180.0 / Math.PI;
    inputs.appliedPercentage = appliedVolts / 12.0;
    inputs.isOpen = falconSim.hasHitUpperLimit();
    inputs.isClosed = falconSim.hasHitLowerLimit();
    inputs.isBlocked = !this.manipulatorSensor.get();

    if (falconSim.hasHitLowerLimit() || falconSim.hasHitUpperLimit()) {
      inputs.statorCurrentAmps = new double[] {STALL_CURRENT_AMPS};
    } else {
      inputs.statorCurrentAmps = new double[] {falconSim.getCurrentDrawAmps()};
    }
  }

  @Override
  public void setPower(double percentage) {
    appliedVolts = -percentage * 12.0;
    falconSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void enableBrakeMode(boolean mode) {
    // not needed for simulation
  }

  @Override
  public void setPosition(double position) {
    setPower(-ManipulatorConstants.MANIPULATOR_POWER);
  }
}
