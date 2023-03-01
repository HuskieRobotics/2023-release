// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements OperatorInterface {
  private final XboxController controller;

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  @Override
  public double getTranslateX() {
    return -controller.getLeftY();
  }

  @Override
  public double getTranslateY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRotate() {
    return -controller.getRightX();
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return new Trigger(controller::getBButton);
  }

  @Override
  public Trigger getResetGyroButton() {
    return new Trigger(controller::getStartButton);
  }

  @Override
  public Trigger getXStanceButton() {
    return new Trigger(controller::getYButton);
  }

  // FIXME: remove after testing elevator
  @Override
  public Trigger getMoveArmToChuteButton() {
    return new Trigger(controller::getLeftBumper);
  }

  @Override
  public Trigger getMoveArmToShelfButton() {
    return new Trigger(controller::getRightBumper);
  }

  @Override
  public Trigger getMoveArmToStorageButton() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public Trigger getMoveArmToLowButton() {
    return new Trigger(controller::getBButton);
  }

  @Override
  public Trigger getMoveArmToMidButton() {
    return new Trigger(controller::getYButton);
  }

  @Override
  public Trigger getMoveArmToHighButton() {
    return new Trigger(controller::getXButton);
  }

  @Override
  public double getRotateArm() {
    return -controller.getLeftY();
  }

  @Override
  public double getMoveElevator() {
    return -controller.getRightX();
  }

  @Override
  public Trigger getEnableManualElevatorControlButton() {
    return new Trigger(() -> controller.getPOV() == 0);
  }

  @Override
  public Trigger getDisableManualElevatorControlButton() {
    return new Trigger(() -> controller.getPOV() == 90);
  }

  @Override
  public Trigger getEnableManualElevatorPresetButton() {
    return new Trigger(() -> controller.getPOV() == 180);
  }

  @Override
  public Trigger getDisableManualElevatorPresetButton() {
    return new Trigger(() -> controller.getPOV() == 270);
  }

  // TODO: Remove when simulated testing is done!
  @Override
  public Trigger getTranslationSlowModeButton() {
    return new Trigger(controller::getLeftBumper);
  }

  @Override
  public Trigger getRotationSlowModeButton() {
    return new Trigger(controller::getRightBumper);
  }

  @Override
  public Trigger getVisionIsEnabledSwitch() {
    // vision is always enabled with Xbox as there is no switch to disable
    return new Trigger(() -> true);
  }

  @Override
  public Trigger getIntakeChuteButton() {
    return new Trigger(controller::getXButton);
  }

  @Override
  public Trigger toggleManipulatorOpenCloseButton() {
    return new Trigger(controller::getRightStickButton);
  }
}
