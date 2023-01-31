package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.Elevator;


public class SetPosition extends CommandBase {
    private Elevator elevator;
    private Position rotation;
    private Position extension;
    /**
     * Constructs a new ExtendClimberToMidRungCommand object.
     *
     * @param subsystem the elevator subsystem this command will control
     * @return 
     */

     public enum  Position{
      CONE_FROM_FLOOR,
      CUBE_FROM_FLOOR,
    }
    public void SetElevatorPosition(Elevator subsystem, Position pos) {
      elevator = subsystem;
      this.rotation = pos;
      this.extension = pos;
      addRequirements(elevator);
    }
  
    /**
     * This method is invoked once when this command is scheduled. It sets the setpoint of the
     * elevator position to slightly above the mid rung. It is critical that this initialization
     * occurs in this method and not the constructor as this command is constructed once when the
     * RobotContainer is created, but this method is invoked each time this command is scheduled.
     */
    @Override
    public void initialize() {
      
      elevator.setPosition(this.rotation, this.extension);
    }
  
    @Override
    public void execute() {

    }
  
    /**
     * This method will be invoked when this command finishes or is interrupted. It stops the motion
     * of the elevator.
     *
     * @param interrupted true if the command was interrupted by another command being scheduled
     */
    @Override
    public void end(boolean interrupted) {
      elevator.stopElevator();
    }
  
    /**
     * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
     * elevator has reached the specified setpoint, which is slightly above the mid rung.
     */
    @Override
    public boolean isFinished() {
      return elevator.atSetpoint();
    }
}
