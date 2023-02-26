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

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveArmButton() { {
    return new Trigger(() -> false);
  }
  
  public default Trigger getTranslationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getVisionIsEnabledSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveToGridButton() {
    return new Trigger(() -> false);
  }

  public default Node getNode() {
    return Node.NODE_INVALID;
  }

  public default Trigger getIntakeShelfRightButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeShelfLeftButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeChuteButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTurboButton() {
    return new Trigger(() -> false);
  }

  public default Trigger toggleManipulatorOpenCloseButton() {
    return new Trigger(() -> false);
  }
}
