package frc.robot.subsystems.elevator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.operator_interface.OperatorInterface;

public class ElevatorIOSim implements ElevatorIOInputs {
  public RobotType robot;
  private ElevatorIOHardware elevatorHardware;
  private static final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4); // CHANGE MOTOR
  private static final double kCarriageMass = 4; // Kg CHANGE
  private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double kElevatorGearing = 10; // NEED ACTUAL VALUES FOR THESE
  private static final double kMinElevatorHeight = 0;
  private static final double kMaxElevatorHeight = Units.inchesToMeters(50);// CHANGE

  private static final int kEncoderAChannel = 0; // CHANGE
  private static final int kEncoderBChannel = 1; // CHANGE, maybe use a constant
  public final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private static final double kElevatorEncoderDistPerPulse =
  2.0 * Math.PI * kElevatorDrumRadius / 4096;

  private final ElevatorSim m_elevatorSim = new ElevatorSim(m_elevatorGearbox,
    kCarriageMass,
    kElevatorDrumRadius,
    kElevatorGearing,
    kMinElevatorHeight,
    kMaxElevatorHeight,
    true, VecBuilder.fill(0.01));
    //https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
  public Mechanism2d elevator;

  public ElevatorIOSim() {

    // the main mechanism object
    robot = Constants.getRobot();











    //  elevator = new Mechanism2d(3, 3);

    // MechanismRoot2d elevator_root = elevator.getRoot("elevator", 2, 0);

    // // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // // off the root node or another ligament object
    // MechanismLigament2d m_elevator =
    //     elevator_root.append(new MechanismLigament2d("elevator", 1, 90));
    // MechanismLigament2d m_wrist =
    //     m_elevator.append(
    //         new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

    // // post the mechanism to the dashboard
    // SmartDashboard.putData("Mech2d", elevator);
  }
  
  public Encoder getElevatorEncoder(){
    return m_encoder;
  }

 

  // public Mechanism2d elevatorSimConstruct(){
  //     MechanismRoot2d elevator_root = elevator.getRoot("elevator", 2, 0);

  //     // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are
  // based
  //     // off the root node or another ligament object
  //     MechanismLigament2d m_elevator = elevator_root.append(new MechanismLigament2d("elevator",
  // 1, 90));
  //     MechanismLigament2d m_wrist = m_elevator.append(
  //             new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

  //      // post the mechanism to the dashboard
  //     SmartDashboard.putData("Mech2d", elevator);

  //     return elevator;
  // }

  // public void updateElevatorSim(){
  //     // post the mechanism to the dashboard
  //     SmartDashboard.putData("Mech2d", elevator);
  // }

}
