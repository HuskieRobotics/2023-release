package frc.robot.subsystems.manipulator;

import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.team254.drivers.TalonFXFactory;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.CANDeviceFinder;
import frc.lib.team3061.util.CANDeviceId.CANDeviceType;
import frc.lib.team6328.util.TunableNumber;

public class ManipulatorIOTalonFX implements ManipulatorIO {
  private TalonFX manipulatorMotor;
  private final DigitalInput manipulatorSensor;
  private boolean isSensorEnabled = true;
  private int stallCount = 0;
  private CANDeviceFinder can;

  private final TunableNumber manipulatorKP =
      new TunableNumber("manipulator/Kp", ManipulatorConstants.MANIPULATOR_KP);
  private final TunableNumber manipulatorKI =
      new TunableNumber("manipulator/Ki", ManipulatorConstants.MANIPULATOR_KI);
  private final TunableNumber manipulatorKD =
      new TunableNumber("manipulator/Kd", ManipulatorConstants.MANIPULATOR_KD);

  public ManipulatorIOTalonFX() {
    this.can = new CANDeviceFinder();
    can.isDevicePresent(
        CANDeviceType.TALON, ManipulatorConstants.MANIPULATOR_MOTOR_ID, "Manipulator Motor");

    this.manipulatorSensor = new DigitalInput(ManipulatorConstants.MANIPULATOR_SENSOR_ID);

    configManipulatorMotor(ManipulatorConstants.MANIPULATOR_MOTOR_ID);
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    inputs.positionDeg = (manipulatorMotor.getSelectedSensorPosition() / 2048.0) * 360;
    inputs.appliedPercentage = manipulatorMotor.getMotorOutputVoltage();
    inputs.statorCurrentAmps = new double[] {manipulatorMotor.getStatorCurrent()};
    inputs.isBlocked = !manipulatorSensor.get() || !isSensorEnabled;

    inputs.isClosed =
        inputs.appliedPercentage > 0
            && inputs.statorCurrentAmps[inputs.statorCurrentAmps.length - 1]
                > ManipulatorConstants.CLOSE_THRESHOLD_CURRENT;

    inputs.isOpen = false;
    if (inputs.appliedPercentage < 0
        && inputs.statorCurrentAmps[0] > ManipulatorConstants.OPEN_THRESHOLD_CURRENT) {
      stallCount++;
      if (stallCount > OPEN_THRESHOLD_ITERATIONS) {
        inputs.isOpen = true;
      }
    } else {
      stallCount = 0;
    }

    if (manipulatorKP.hasChanged() || manipulatorKI.hasChanged() || manipulatorKD.hasChanged()) {
      manipulatorMotor.config_kP(0, manipulatorKP.get());
      manipulatorMotor.config_kI(0, manipulatorKI.get());
      manipulatorMotor.config_kD(0, manipulatorKD.get());
    }
  }

  @Override
  public void setPower(double percentage) {
    if (percentage > 0) {
      manipulatorMotor.configStatorCurrentLimit(
          ManipulatorConstants.MANIPULATOR_CURRENT_LIMIT_CLOSE);
      manipulatorMotor.set(TalonFXControlMode.PercentOutput, percentage);
    } else if (percentage < 0) {
      manipulatorMotor.configStatorCurrentLimit(
          ManipulatorConstants.MANIPULATOR_CURRENT_LIMIT_OPEN);
      manipulatorMotor.set(TalonFXControlMode.PercentOutput, percentage);
    } else if (percentage == 0) {
      manipulatorMotor.set(TalonFXControlMode.PercentOutput, percentage);
    }
  }

  private void configManipulatorMotor(int manipulatorMotorID) {
    TalonFXFactory.Configuration manipulatorMotorConfig = new TalonFXFactory.Configuration();
    manipulatorMotorConfig.INVERTED = ManipulatorConstants.MANIPULATOR_MOTOR_INVERTED;
    manipulatorMotorConfig.BRUSHLESS_CURRENT_STATUS_FRAME_RATE_MS = 9;
    manipulatorMotorConfig.SLOT0_KP = manipulatorKP.get();
    manipulatorMotorConfig.SLOT0_KI = manipulatorKI.get();
    manipulatorMotorConfig.SLOT0_KD = manipulatorKD.get();
    manipulatorMotorConfig.NEUTRAL_MODE = NeutralMode.Brake;

    manipulatorMotor =
        TalonFXFactory.createTalon(
            manipulatorMotorID, RobotConfig.getInstance().getCANBusName(), manipulatorMotorConfig);

    manipulatorMotor.setSelectedSensorPosition(0);
  }
}
