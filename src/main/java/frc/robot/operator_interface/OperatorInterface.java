// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default double getRotateArm() {
    return 0.0;
  }

  public default double getMoveElevator() {
    return 0.0;
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTranslateSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getInterruptButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getGroundCubeIntakeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotateSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAlignIntakeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDeployIntakeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRetractIntakeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmChuteButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmShelfButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmSecureGamePieceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmLowButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmHighButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmMiddleButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getToggleRollerButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getToggleManipulatorButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getEnableManualArmButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDisableManualArmButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDisableArmPresetsButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getEnableArmPresetsButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getYPLEDToggleButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeShelfRightButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeShelfLEftButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeChuteButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeGroundConeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveToGridButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getToggleVisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getToggleAutoDriveButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAutoBalanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getToggleManipulatorSensorButton() {
    return new Trigger(() -> false);
  }

  public default double getScoringLevelSwitchValue() {
    return 0.0;
  }

  public default double getScoringColumnSwitchValue() {
    return 0.0;
  }

  public default double getScoringGridSwitchValue() {
    return 0.0;
  }

  public default Trigger getHybridLeftMiddleGridButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getHybridMiddleRightGridButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getHybridLeftMiddleColumnButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getHybridMiddleRightColumnButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getHybridHighMiddleLevelButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getHybridMiddleLowLevelButton() {
    return new Trigger(() -> false);
  }
}
