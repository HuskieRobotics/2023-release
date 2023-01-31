package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import java.util.EnumSet;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetPosition.position;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {

  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private final ElevatorIOSim sim = new ElevatorIOSim();
  public double rotationSetpoint = 0.0;
  public double extensionSetpoint = 0.0;

  public boolean isControlEnabled = false;

  private Encoder m_Encoder = sim.getElevatorEncoder();
  public ElevatorIO io;
  public int valueListenerHandle;


  public Elevator(ElevatorIO io) {
    this.io = io;

    final DoubleSubscriber ySub;
    final AtomicReference<Double> yValue = new AtomicReference<Double>();
    
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
     // get the default instance of NetworkTables
     NetworkTableInstance instListner = NetworkTableInstance.getDefault();

     ySub = instListner.getDoubleTopic("Y").subscribe(0.0);

     valueListenerHandle = instListner.addListener(
      ySub,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        // can only get doubles because it's a DoubleSubscriber, but
        // could check value.isDouble() here too
        yValue.set(event.valueData.value.getDouble());
      });

    if (DEBUGGING) {
      tab.add("elevator", this);
      tab.addNumber("Encoder", this::getExtensionElevatorEncoderHeight);
    }

    if (TESTING) {
      tab.add("Set Position", new SetPosition(this));     // FIXME create two sliders for rotation and extension
    }

    if(TUNING){
      io.setControlEnabled(true);

      tab.addNumber("Rotation Closed Loop Target", this::getSetpoint); // FIXME
      tab.addNumber("Rotation Closed Loop Error", () -> inputs.rotationClosedLoopError);
      tab.addNumber("Rotation Velocity", () -> inputs.rotationVelocity);
      tab.addNumber("Rotation Right Motor Volts", () -> inputs.rotationAppliedVolts);

      tab.addNumber("Extension Closed Loop Target", this::getSetpoint); // FIXME
      tab.addNumber("Extension Closed Loop Error", () -> inputs.rotationClosedLoopError);
      tab.addNumber("Extension Velocity", () -> inputs.rotationVelocity);
      tab.addNumber("Extension Left Motor Volts", () -> inputs.extensionAppliedVolts);

      tab.add("Extension Motor", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -1, "max", 1))
          .getEntry()
          .addListener(
              event -> this.setElevatorExtensionMotorPower(event.getEntry().getValue().getDouble()),  
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);  // FIXME

      tab.add("Extension Position Setpoint", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", MAX_EXTENSION_POSITION))
          .getEntry()
          .addListener(
              event -> this.setElevatorExtension(event.getEntry().getValue().getDouble(), true),  
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Extension Position PID F", EXTENSION_POSITION_PID_F)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureExtensionKF(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Extension Position PID P", EXTENSION_POSITION_PID_P)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureExtensionKP(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Extension Position PID I", EXTENSION_POSITION_PID_I)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureExtensionKI(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Extension Position PID D", EXTENSION_POSITION_PID_D)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureExtensionKD(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
  }

  @Override
  public void periodic() {
  io.updateInputs(inputs);
  Logger.getInstance().processInputs("Elevator", inputs);
  }

  public void setElevatorExtensionMotorPower(double power) {
    if(
      (this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500 && power > 0) ||
      (this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500 && power < 0)) {
      stopExtension();
    }
    else {
      io.setExtensionMotorPercentage(power);
    }
  }
  public void setElevatorRotationMotorPower(double power) {
    if(
      (this.getRotationElevatorEncoderAngle() > MAX_ROTATION_POSITION - 2500 && power > 0) ||
      (this.getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500 && power < 0)) {
      stopExtension();
    }
    else {
      io.setRotationMotorPercentage(power);
    }
  }

  public void setElevatorExtension(position extension) {
    if(
      (this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500 && power > 0) ||
      (this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500 && power < 0)) {
      stopExtension();
    }
    else{
      if(getRotationElevatorEncoderAngle() < MIN_ELEVATOR_EXTENSION_ANGLE){
        setRotationMotorPosition(elevator, MIN_ELEVATOR_EXTENSION_ANGLE);
      }
      this.extensionSetpoint = extension;
      io.setExtensionnPosition(extension, ARBITRARY_FEED_FORWARD_EXTENSION);
    }
  }

  public void setElevatorRotation(position rotation) {
    if(
      (this.getRotationElevatorEncoderAngle() > MAX_ROTATION_POSITION - 2500 && power > 0) ||
      (this.getRotationElevatorEncoderAngle() < MIN_ROTATION_POSITION + 2500 && power < 0)) {
      this.stopExtension();
    }
    else {
      io.setRotationPosition(rotation, ARBITRARY_FEED_FORWARD_ROTATION);
      this.rotationSetpoint = rotation;
    }
  }

  public boolean atExtensionSetpoint() {
    return Math.abs(this.inputs.extensionPosition - extensionSetpoint) < ELEVATOR_EXTENSION_POSITION_TOLERANCE; 
  }

  public boolean atRotationSetpoint() {
    return Math.abs(this.inputs.rotationPosition - rotationSetpoint) < ELEVATOR_ROTATION_POSITION_TOLERANCE;
  }

  public void stopExtension() {
    this.io.setExtensionMotorPercentage(0.0);
  }

  public void stopRotation() {
    this.io.setRotationMotorPercentage(0.0);
  }

  public void stopElevator() {
    this.stopExtension();
    this.stopRotation();
  }
  
  public double getExtensionElevatorEncoderHeight() {
     return this.inputs.extensionPosition;
  }

  public double getRotationElevatorEncoderAngle() {
    return this.inputs.rotationPosition;
 }

  public void setPosition(position rotation, position extension) {
    this.setElevatorExtension(extension);
    this.setElevatorRotation(rotation);
  }

  public boolean atSetpoint() { 
    return this.atRotationSetpoint() && this.atExtensionSetpoint();
  }

  public void setControlEnabled(){
    this.isControlEnabled = true;
  }

  public boolean nearExtensionMaximum() {
    return this.getExtensionElevatorEncoderHeight() > MAX_EXTENSION_POSITION - 2500; // FIXME
  }

  public boolean nearExtensionMinimum() {
    return this.getExtensionElevatorEncoderHeight() < MIN_EXTENSION_POSITION + 2500; // FIXME
  }
}
