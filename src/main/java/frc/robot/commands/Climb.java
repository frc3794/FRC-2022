// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {

  private final XboxController m_controller = new XboxController(1);
  private final Climber m_climber;

  /** Creates a new MoveElevator. */
  public Climb(Climber climber) {
    this.m_climber = climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.slideIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getLeftY() > 0.2) {
      m_climber.extendLeftArm();
    } else if (m_controller.getLeftY() < -0.2) {
      m_climber.contractLeftArm();
    } else {
      m_climber.stopLeftArm();
    }

    if (m_controller.getRightY() > 0.2) {
      m_climber.extendRightArm();
    } else if (m_controller.getRightY() < -0.2) {
      m_climber.contractRightArm();
    } else {
      m_climber.stopRightArm();
    }

    if (m_controller.getLeftBumper()) {
      m_climber.slideIn();
    }

    if (m_controller.getRightBumper()) {
      m_climber.slideOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
