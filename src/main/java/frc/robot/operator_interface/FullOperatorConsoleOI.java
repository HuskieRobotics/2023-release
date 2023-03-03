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
    this.operatorPanelButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
      operatorPanelButtons[i] = operatorPanel.button(i);
    }
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
  public Trigger getMoveArmToChuteButton() {
    return new Trigger(operatorController::getLeftBumper);
  }

  @Override
  public Trigger getMoveArmToShelfButton() {
    return new Trigger(operatorController::getRightBumper);
  }

  @Override
  public Trigger getMoveArmToStorageButton() {
    return new Trigger(operatorController::getAButton);
  }

  @Override
  public Trigger getMoveArmToLowButton() {
    return new Trigger(operatorController::getBButton);
  }

  @Override
  public Trigger getMoveArmToMidButton() {
    return new Trigger(operatorController::getYButton);
  }

  @Override
  public Trigger getMoveArmToHighButton() {
    return new Trigger(operatorController::getXButton);
  }

  @Override
  public double getRotateArm() {
    return -operatorController.getLeftY();
  }

  @Override
  public double getMoveElevator() {
    return -operatorController.getRightX();
  }

  @Override
  public Trigger getEnableManualElevatorControlButton() {
    return new Trigger(() -> operatorController.getPOV() == 0);
  }

  @Override
  public Trigger getDisableManualElevatorControlButton() {
    return new Trigger(() -> operatorController.getPOV() == 180);
  }

  @Override
  public Trigger getEnableManualElevatorPresetButton() {
    return new Trigger(() -> operatorController.getPOV() == 270);
  }

  @Override
  public Trigger getDisableManualElevatorPresetButton() {
    return new Trigger(() -> operatorController.getPOV() == 90);
  }

  @Override
  public Trigger getConeCubeLEDTriggerButton() {
    return operatorPanelButtons[1];
  }

  @Override
  public Trigger getTranslationSlowModeButton() {
    return translateJoystickButtons[2];
  }

  @Override
  public Trigger getRotationSlowModeButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger getVisionIsEnabledSwitch() {
    return operatorPanelButtons[10];
  }

  @Override
  public Trigger getMoveToGridButton() {
    return operatorPanelButtons[9];
  }

  private double getScoringGridSwitchValue() {
    if (operatorPanelButtons[3].getAsBoolean()) {
      return -1;
    } else if (operatorPanelButtons[4].getAsBoolean()) {
      return 1;
    } else {
      return 0;
    }
  }

  private double getScoringColumnSwitchValue() {
    return operatorPanel.getX();
  }

  private double getScoringLevelSwitchValue() {
    return operatorPanel.getY();
  }

  @Override
  public Node getNode() {
    if (this.getScoringGridSwitchValue() == -1) {
      if (this.getScoringColumnSwitchValue() == -1) {
        return Node.NODE_1;
      } else if (this.getScoringColumnSwitchValue() == 0) {
        return Node.NODE_2;
      } else {
        return Node.NODE_3;
      }
    } else if (this.getScoringGridSwitchValue() == 0) {
      if (this.getScoringColumnSwitchValue() == -1) {
        return Node.NODE_4;
      } else if (this.getScoringColumnSwitchValue() == 0) {
        return Node.NODE_5;
      } else {
        return Node.NODE_6;
      }
    } else {
      if (this.getScoringColumnSwitchValue() == -1) {
        return Node.NODE_7;
      } else if (this.getScoringColumnSwitchValue() == 0) {
        return Node.NODE_8;
      } else {
        return Node.NODE_9;
      }
    }
  }

  @Override
  public Trigger getIntakeShelfRightButton() {
    return operatorPanelButtons[5];
  }

  @Override
  public Trigger getIntakeShelfLeftButton() {
    return operatorPanelButtons[6];
  }

  @Override
  public Trigger getIntakeChuteButton() {
    return operatorPanelButtons[7];
  }

  @Override
  public Trigger getTurboButton() {
    return translateJoystickButtons[4];
  }

  @Override
  public Trigger toggleManipulatorOpenCloseButton() {
    return new Trigger(operatorController::getRightStickButton);
  }

}
