// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class DualJoysticksOI implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final CommandJoystick rotateJoystick;
  public final JoystickButton[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;

  public DualJoysticksOI(int translatePort, int rotatePort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new JoystickButton[13];
    this.rotateJoystickButtons = new Trigger[13];
  }

  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getResetGyroButton() {
    return rotateJoystickButtons[3];
  }

  @Override
  public Trigger getXStanceButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger getMoveArmButton() {
    return rotateJoystickButtons[4];
  }
}
