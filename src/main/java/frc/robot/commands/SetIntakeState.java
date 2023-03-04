package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This command, when executed, instructs the intake subsystem to rotate the intake to the specified
 * position and, based on that position, runs the roller at the corresponding speed.
 *
 * <p>Requires: the Intake subsystem
 *
 * <p>Finished When: the intake is at the specified position
 *
 * <p>At End: stop the rotation (relying on the gearbox to hold the position) and leaves the roller
 * running
 */
public class SetIntakeState extends CommandBase {
  private Intake intake;
  private double rotation;
  private double rollerSpeed;
  private Position position;
  private LoggedDashboardChooser<Position> intakeChooser;

  /**
   * Constructs a new SetIntakeState command that changes based on the given state.
   *
   * @param subsystem the intake subsystem this command will control
   * @return
   */
  public SetIntakeState(Intake subsystem, Position position) {
    intake = subsystem;
    this.position = position;

    addRequirements(intake);
  }

  public SetIntakeState(Intake subsystem) {
    intake = subsystem;

    // this.intakeChooser = intakeChooser;
    // add this after we remove the testing section as a parameter LoggedDashboardChooser<Position>
    // intakeChooser

    // FIXME: remove after testing
    this.intakeChooser = new LoggedDashboardChooser<>("Intake Position");
    this.intakeChooser.addDefaultOption("CHARGE_STATION", Position.CHARGE_STATION);
    this.intakeChooser.addOption("RETRACTED", Position.RETRACTED);
    this.intakeChooser.addOption("SCORING", Position.SCORING);
    this.intakeChooser.addOption("CUBE_INTAKE", Position.CUBE_INTAKE);

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    if (this.intakeChooser != null) {
      this.position = intakeChooser.get();
    }

    switch (this.position) {
      case CHARGE_STATION:
        this.rollerSpeed = IntakeConstants.CHARGE_STATION_ROLLER_POWER;
        this.rotation = IntakeConstants.CHARGE_STATION_ROTATION;
        break;
      case RETRACTED:
        this.rollerSpeed = IntakeConstants.RETRACTED_ROLLER_POWER;
        this.rotation = IntakeConstants.RETRACTED_ROTATION;
        break;
      case SCORING:
        this.rollerSpeed = IntakeConstants.SCORING_ROLLER_POWER;
        this.rotation = IntakeConstants.SCORING_ROTATION;
        break;
      case CUBE_INTAKE:
        this.rollerSpeed = IntakeConstants.CUBE_INTAKE_ROLLER_POWER;
        this.rotation = IntakeConstants.CUBE_INTAKE_ROTATION;
        break;
      case PUSH_CONE_CUBE:
        this.rollerSpeed = IntakeConstants.PUSH_CONE_CUBE_ROLLER_POWER;
        this.rotation = IntakeConstants.PUSH_CONE_CUBE_ROTATION;
        break;
    }

    intake.setRotationPosition(this.rotation);
    intake.setRollerMotorPercentage(this.rollerSpeed);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    intake.stopRotation();
  }

  /** This method is invoked at the end of each Command Scheduler iteration. */
  @Override
  public boolean isFinished() {
    return intake.atSetpoint();
  }
}
