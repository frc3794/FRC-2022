// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveDrivetrain extends CommandBase {
  
  private final Drivetrain m_drivetrain;
  private final XboxController m_controller = Robot.getDrivetrainController();

  public MoveDrivetrain(Drivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() { }

  @Override
  public void execute() {
    double fwd = 0, rot = 0;

    double rightTrigger = Math.abs(m_controller.getRightTriggerAxis());
    double leftTrigger = Math.abs(m_controller.getLeftTriggerAxis());

    if (rightTrigger > 0.2) {
      fwd = rightTrigger;
    } else if (leftTrigger > 0.2) {
      fwd = -leftTrigger;
    } else {
      fwd = 0;
    }

    double rightJoystick = m_controller.getRightX();
    double leftJoystick = m_controller.getLeftX();

    if (Math.abs(rightJoystick) > 0.2) {
      rot = rightJoystick * 0.8;
    } else if (Math.abs(leftJoystick) > 0.2) {
      rot = leftJoystick;
    } else {
      rot = 0;
    }

    fwd *= DrivetrainConstants.kSensibilityPercent;
    rot *= DrivetrainConstants.kSensibilityPercent * -0.8;

    m_drivetrain.arcadeDrive(fwd, rot);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
