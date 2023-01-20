package frc.robot.subsystems.TimeOfFlightSensor;


import com.fasterxml.jackson.databind.ser.std.NumberSerializers.IntLikeSerializer;
import com.playingwithfusion.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSensor extends SubsystemBase{
    private static final int TIME_OF_FLIGHT_CAN_ID = 1; // random number for the CAN_ID we can change it late r

    TimeOfFlight m_Sensor = new TimeOfFlight(TIME_OF_FLIGHT_CAN_ID); 

    public TimeOfFlightSensor(){
        ShuffleboardTab tab = Shuffleboard.getTab("Time Of Flight Sensor"); 

        //tab.add("Distance", m_Sensor.getRange()); // Returns the range 
        //tab.add("Status", m_Sensor.getStatus()); // sends the status 
        //tab.add("Speed", m_Sensor.getSpeed()); // sends the speed 
        //tab.add("Temperature", m_Sensor.getTemperature()); // sends the temperature 
    }

    

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        System.out.println(m_Sensor.getRange()); 
        super.periodic();
    }
}
