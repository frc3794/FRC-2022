// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wint3794.frc;

import org.wint3794.frc.Constants.DrivetrainConstants;
import org.wint3794.frc.commands.BackwardCommand;
import org.wint3794.frc.commands.ForwardCommand;
import org.wint3794.frc.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();

  private final ForwardCommand m_autoCommand = new ForwardCommand(m_drivetrain);

  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_subsystemController = new XboxController(1);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public void teleop() {
    double fwd = 0, rot = 0;

    double rightTrigger = m_driveController.getRightTriggerAxis();
    double leftTrigger = m_driveController.getLeftTriggerAxis();

    if (rightTrigger > 0.2) {
      fwd = rightTrigger;
    } else if (leftTrigger > 0.2) {
      fwd = -leftTrigger;
    } else {
      fwd = 0;
    }

    double rightJoystick = m_driveController.getRightX();
    double leftJoystick = m_driveController.getLeftX();

    if (Math.abs(rightJoystick) > 0.2) {
      rot = rightJoystick * 0.8;
    } else if (Math.abs(leftJoystick) > 0.2) {
      rot = leftJoystick;
    } else {
      rot = 0;
    }

    fwd *= DrivetrainConstants.kSensibilityPercent;
    rot *= DrivetrainConstants.kSensibilityPercent * 0.8;

    m_drivetrain.arcadeDrive(fwd, rot);
  }
}
