package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.GrabGamePiece;
import frc.robot.commands.ReleaseGamePiece;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {

  private static final boolean TESTING = false;
  private final ManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

  public Manipulator(ManipulatorIO io) {
    this.io = io;
    ShuffleboardTab tab = Shuffleboard.getTab("Manipulator");
    tab.addBoolean("isBlocked", this::isBlocked);

    if (TESTING) {
      tab.add("Open Manipulator", new ReleaseGamePiece(this));
      tab.add("Close Manipulator", new GrabGamePiece(this));
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

  public void enableBrakeMode(boolean mode) {
    io.enableBrakeMode(mode);
  }

  public void openPosition() {
    io.setPosition(0);
  }

  public void openPower() {
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

  public void stop() {
    io.setPower(0);
  }
}
