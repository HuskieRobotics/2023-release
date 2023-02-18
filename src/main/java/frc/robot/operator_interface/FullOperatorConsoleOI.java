// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class FullOperatorConsoleOI implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final Trigger[] translateJoystickButtons;

  private final CommandJoystick rotateJoystick;
  private final Trigger[] rotateJoystickButtons;

  private final XboxController operatorController;

  private final CommandJoystick operatorPanel;
  private final Trigger[] operatorPanelButtons;

  public FullOperatorConsoleOI(
      int translatePort, int rotatePort, int operatorControllerPort, int operatorPanelPort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);
    operatorController = new XboxController(operatorControllerPort);
    operatorPanel = new CommandJoystick(operatorPanelPort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];
    this.operatorPanelButtons = new Trigger[17];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
    for (int i = 1; i < operatorPanelButtons.length; i++) {
      operatorPanelButtons[i] = operatorPanel.button(i);
    }
  }

  public boolean getOperatorControllerRightTrigger() {
    return operatorController.getRightTriggerAxis() > 0.25;
  }

  public boolean getOperatorControllerLeftTrigger() {
    return operatorController.getLeftTriggerAxis() > 0.25;
  }

  public boolean getDPadUp() {
    return operatorController.getPOV() == 0;
  }

  public boolean getDPadDown() {
    return operatorController.getPOV() == 5;
  }

  public boolean getDPadLeft() {
    return operatorController.getPOV() == 7;
  }

  public boolean getDPadRight() {
    return operatorController.getPOV() == 3;
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
  public double getRotateArm() {
    return operatorController.getRightY();
  }

  @Override
  public double getMoveElevator() {
    return operatorController.getLeftY();
  }

  // Translate Joystick

  @Override
  public Trigger getResetGyroButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger getTranslateSlowModeButton() {
    return translateJoystickButtons[2];
  }

  @Override
  public Trigger getInterruptButton() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[5];
  }

  // Rotate Joystick
  @Override
  public Trigger getGroundCubeIntakeButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getRotateSlowModeButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger getAlignIntakeButton() {
    return rotateJoystickButtons[3];
  }

  @Override
  public Trigger getXStanceButton() {
    return rotateJoystickButtons[4];
  }

  // Operator Controller
  @Override
  public Trigger getDeployIntakeButton() {
    return new Trigger(this::getOperatorControllerLeftTrigger);
  }

  @Override
  public Trigger getRetractIntakeButton() {
    return new Trigger(this::getOperatorControllerRightTrigger);
  }

  @Override
  public Trigger getArmChuteButton() {
    return new Trigger(operatorController::getLeftBumper);
  }

  @Override
  public Trigger getArmShelfButton() {
    return new Trigger(operatorController::getRightBumper);
  }

  @Override
  public Trigger getArmSecureGamePieceButton() {
    return new Trigger(operatorController::getAButton);
  }

  @Override
  public Trigger getArmLowButton() {
    return new Trigger(operatorController::getBButton);
  }

  @Override
  public Trigger getArmHighButton() {
    return new Trigger(operatorController::getXButton);
  }

  @Override
  public Trigger getArmMiddleButton() {
    return new Trigger(operatorController::getYButton);
  }

  @Override
  public Trigger getToggleRollerButton() {
    return new Trigger(operatorController::getLeftStickButton);
  }

  @Override
  public Trigger getToggleManipulatorButton() {
    return new Trigger(operatorController::getRightStickButton);
  }

  @Override
  public Trigger getEnableManualArmButton() {
    return new Trigger(this::getDPadUp);
  }

  @Override
  public Trigger getDisableManualArmButton() {
    return new Trigger(this::getDPadDown);
  }

  @Override
  public Trigger getEnableArmPresetsButton() {
    return new Trigger(this::getDPadLeft);
  }

  @Override
  public Trigger getDisableArmPresetsButton() {
    return new Trigger(this::getDPadRight);
  }

  // Operator Console
  @Override
  public Trigger getYPLEDToggleButton() {
    return operatorPanelButtons[1];
  }

  public Trigger getIntakeShelfRightButton() {
    return operatorPanelButtons[5];
  }

  public Trigger getIntakeShelfLeftButton() {
    return operatorPanelButtons[6];
  }

  @Override
  public Trigger getIntakeChuteButton() {
    return operatorPanelButtons[7];
  }

  @Override
  public Trigger getIntakeGroundConeButton() {
    return operatorPanelButtons[8];
  }

  @Override
  public Trigger getMoveToGridButton() {
    return operatorPanelButtons[9];
  }

  @Override
  public Trigger getToggleVisionButton() {
    return operatorPanelButtons[10];
  }

  @Override
  public Trigger getToggleAutoDriveButton() {
    return operatorPanelButtons[11];
  }

  @Override
  public Trigger getAutoBalanceButton() {
    return operatorPanelButtons[12];
  }

  @Override
  public Trigger getToggleManipulatorSensorButton() {
    return operatorPanelButtons[13];
  }

  @Override
  public double getScoringLevelSwitchValue() {
    if(operatorPanelButtons[3].getAsBoolean()) {
      return 1;
    } else if(operatorPanelButtons[4].getAsBoolean()) {
      return -1;
    } else {
      return 0;
    }
  }

  @Override
  public double getScoringColumnSwitchValue() {
    return operatorPanel.getX();
  }

  @Override
  public double getScoringGridSwitchValue() {
    return operatorPanel.getY();
  }

  // TODO: Check if values correspond to the correct values on the switches (based on method names)
  // @Override
  // public Trigger getHybridLeftMiddleGridButton() {
  //   return operatorPanelButtons[4];
  // }

  // @Override
  // public Trigger getHybridMiddleRightGridButton() {
  //   return operatorPanelButtons[16];
  // }

  // @Override
  // public Trigger getHybridLeftMiddleColumnButton() {
  //   return operatorPanelButtons[3];
  // }

  // @Override
  // public Trigger getHybridMiddleRightColumnButton() {
  //   return operatorPanelButtons[15];
  // }

  // @Override
  // public Trigger getHybridHighMiddleLevelButton() {
  //   return operatorPanelButtons[2];
  // }

  // @Override
  // public Trigger getHybridMiddleLowLevelButton() {
  //   return operatorPanelButtons[14];
  // }
}
