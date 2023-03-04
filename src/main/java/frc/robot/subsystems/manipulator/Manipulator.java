package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ReleaseGamePiece;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {

  private static final boolean TESTING = false;
  private final ManipulatorIO io;
  private boolean isManipulatorSensorEnabled;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

  public Manipulator(ManipulatorIO io) {
    this.io = io;
    ShuffleboardTab tab = Shuffleboard.getTab("Manipulator");
    tab.addBoolean("isBlocked", this::isBlocked);
    isManipulatorSensorEnabled = true;

    if (TESTING) {
      tab.add("Open Manipulator", new ReleaseGamePiece(this));
      // tab.add("Close Manipulator", new GrabGamePiece(this, null));
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Manipulator", inputs);
  }

  public void setManipulatorPower(double speed) {
    io.setPower(speed);
  }

  public void open() {
    io.setPower(-ManipulatorConstants.MANIPULATOR_POWER);
  }

  public void close() {
    io.setPower(ManipulatorConstants.MANIPULATOR_POWER);
  }

  public boolean isClosed() {
    return inputs.isClosed;
  }

  public boolean isOpened() {
    return inputs.isOpen;
  }

  public boolean isBlocked() {
    return inputs.isBlocked;
  }

  public void enableManipulatorSensor(boolean enable) {
    this.isManipulatorSensorEnabled = enable;
  }

  public boolean isManipulatorSensorEnabled() {
    return this.isManipulatorSensorEnabled;
  }

  public void stop() {
    io.setPower(0);
  }

  public boolean isCanDevicePresent() {
    return io.isCanDevicePresent();
  }
}
