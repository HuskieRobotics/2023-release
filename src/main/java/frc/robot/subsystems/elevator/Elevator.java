package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
// import frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {

  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private final ElevatorIOSim sim = new ElevatorIOSim();

  private Encoder m_Encoder = sim.getElevatorEncoder();
  public ElevatorIO io;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    // Logger.getInstance().processInputs(SUBSYSTEM_NAME, inputs);

    // double pitch = inputs.pitch;
  }

  public void setElevatorMotorPower(double power) {}

  /**
   * @param desiredEncoderPosition this may not be an encoder, check in teh future
   */
  public void setElevatorMotorPosition(double desiredEncoderPosition) {}

  public boolean atSetpoint() {
    return true;
  }

  public void stopElevator() {
    io.setMotorPercentage(0.0);
  }

  public void enableElevatorControl() {
    io.setControlEnabled(true);
  }

  public void disableElevatorControl() {
    io.setControlEnabled(false);
  }
  
  // private double getElevatorEncoderHeight() {
  //    return inputs.rightPosition;
  // }
  /** CHECK THIS - not sure about about encoder */
  private double getSetPoint() {
    return 10;
  }
}
