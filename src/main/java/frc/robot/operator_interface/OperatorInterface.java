// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.*;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {
  public enum Node {
    NODE_INVALID,
    NODE_1,
    NODE_2,
    NODE_3,
    NODE_4,
    NODE_5,
    NODE_6,
    NODE_7,
    NODE_8,
    NODE_9
  }

  public enum GridRow {
    BOTTOM,
    MIDDLE,
    TOP
  }

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseToVisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDisableArmBackupButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveArmToShelfButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveArmToStorageButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveArmToStorageBackupButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveArmToLowButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveArmToMidButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveArmToHighButton() {
    return new Trigger(() -> false);
  }

  public default double getRotateArm() {
    return 0.0;
  }

  public default double getMoveElevator() {
    return 0.0;
  }

  public default Trigger getEnableManualElevatorControlButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDisableManualElevatorControlButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getEnableManualElevatorPresetButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDisableManualElevatorPresetButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getConeCubeLEDTriggerButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTranslationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDisableArmButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTurn180Button() {
    return new Trigger(() -> false);
  }

  public default Trigger getVisionIsEnabledSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveToGridButton() {
    return new Trigger(() -> false);
  }

  public default GridRow getGridRow() {
    return GridRow.BOTTOM;
  }

  public default Node getNode() {
    return Node.NODE_INVALID;
  }

  public default Trigger getIntakeShelfGridSideButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeShelfWallSideButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeShelfGridSideBackupButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeShelfWallSideBackupButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeChuteButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTurboButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getToggleManipulatorOpenCloseButton() {
    return new Trigger(() -> false);
  }

  public default boolean getManualManipulatorClose() {
    return false;
  }

  public default Trigger getToggleManipulatorSensorButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveToGridEnabledSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeDeployButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeRetractButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getToggleIntakeRollerButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPositionIntakeToPushCubeCone() {
    return new Trigger(() -> false);
  }

  public default Trigger getAutoBalanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeGroundConeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getInterruptAll() {
    return new Trigger(() -> false);
  }

  public default Trigger getAutoZeroExtensionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getReleaseTriggerButton() {
    return new Trigger(() -> false);
  }
}
