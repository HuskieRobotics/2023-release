package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class ElevatorIOSim extends TimedRobot {
  public RobotType robot;
  private ElevatorIOTalonFX elevatorHardware;
  private static final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4); // CHANGE MOTOR
  private static final double kCarriageMass = 4; // Kg CHANGE
  private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double kElevatorGearing = 10; // NEED ACTUAL VALUES FOR THESE
  private static final double kMinElevatorHeight = 0;
  private static final double kMaxElevatorHeight = Units.inchesToMeters(50); // CHANGE
  private static final int kMotorPort = 0; // CHANGE
  private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);

  private static final int kEncoderAChannel = 0; // CHANGE
  private static final int kEncoderBChannel = 1; // CHANGE, maybe use a constant
  public final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private static final double kElevatorEncoderDistPerPulse =
      2.0 * Math.PI * kElevatorDrumRadius / 4096;

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          kCarriageMass,
          kElevatorDrumRadius,
          kElevatorGearing,
          kMinElevatorHeight,
          kMaxElevatorHeight,
          true,
          VecBuilder.fill(0.01));
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html

  private final Mechanism2d elevator = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = elevator.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator", Units.metersToInches(m_elevatorSim.getPositionMeters()), 90));

  public ElevatorIOSim() {

    // the main mechanism object
    robot = Constants.getRobot();
  }

  public Encoder getElevatorEncoder() {
    return m_encoder;
  }

  @Override
  public void robotInit() {
    // TODO Auto-generated method stub
    m_encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator Sim in the simulator, select Network Tables -> SmartDashboard ->
    // Elevator Sim
    SmartDashboard.putData("Elevator Sim", elevator);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    m_elevatorMech2d.setLength(Units.metersToInches(m_elevatorSim.getPositionMeters()));
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_motor.set(0.0);
  }
}
