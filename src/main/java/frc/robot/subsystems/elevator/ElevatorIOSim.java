package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorIOSim implements ElevatorIOInputs {
  public Mechanism2d elevator;

  public ElevatorIOSim() {

    // the main mechanism object
    Mechanism2d elevator = new Mechanism2d(3, 3);

    MechanismRoot2d elevator_root = elevator.getRoot("elevator", 2, 0);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    MechanismLigament2d m_elevator =
        elevator_root.append(new MechanismLigament2d("elevator", 1, 90));
    MechanismLigament2d m_wrist =
        m_elevator.append(
            new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

    // post the mechanism to the dashboard
    SmartDashboard.putData("Mech2d", elevator);
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
