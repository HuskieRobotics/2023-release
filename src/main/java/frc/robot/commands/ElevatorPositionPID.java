package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorPositionPID extends PIDCommand{
    
    private Elevator elevator;
    private double setPoint;
    
    public ElevatorPositionPID(Elevator subsystem, double position) {
        super(new PIDController(0, 0, 0), subsystem::getElevatorGyro, position, (output) -> subsystem.setElevatorRotationMotorPower(output));
        addRequirements(subsystem);
        elevator = subsystem;
        setPoint = position;
    }
    @Override
    public void end(boolean interrupted){
        elevator.stopExtension();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getElevatorGyro() - setPoint) <= 1;
    }

    
}
