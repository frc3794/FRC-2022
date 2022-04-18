// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {

  private final XboxController m_controller = Robot.getSubsystemController();
  private final Climber m_climber;
  private int extend = 4;
  private boolean upup1 = false;
  private boolean upup2 = false;

  public Climb(Climber climber) {
    this.m_climber = climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.slideIn();
  }
  
@Override
public void execute() {
  if (m_controller.getPOV() != -1) {
    if (m_controller.getPOV() == 0) {
      extend = 1;
    } else if (m_controller.getPOV() == 180) {
      extend = 0;
    } else if (m_controller.getPOV() == 90) {
      extend = 2;
    }
    return;
  }
  
  if (m_controller.getRightY() > 0.2) {
    m_climber.contractRightArm();
    m_climber.contractLeftArm();
    return;
  } else if (m_controller.getRightY() < -0.2) {
    m_climber.extendRightArm();
    m_climber.extendLeftArm();
    return;
  } else {
    m_climber.stopRightArm();
    m_climber.stopLeftArm();
  }

  if (m_controller.getLeftBumper()) {
    m_climber.slideIn();
  }

  if (m_controller.getRightBumper()) {
    m_climber.slideOut();
  }

  //Paolos

  if (extend == 1) { 
    upup1 = m_climber.extendLeftArm();
    upup2 = m_climber.extendRightArm();
    if (upup1 && upup2) {
      extend = 4;
    }
    return;
  } else if (extend == 0) {
    boolean down = false;
    boolean down2 = false;
    while (!down || !down2) {
      down = m_climber.contractLeftArm();
      down2 = m_climber.contractRightArm();
    }
    extend = 4;
    return;
  } else if (extend == 2) {
    int x = 0;
    while (x < 5000) {
      m_climber.extendLeftArm();
      m_climber.extendRightArm();
      x ++;
    }
    m_climber.slideOut();
    boolean up = false;
    boolean up2 = false;
    while (!up || !up2) { 
      up = m_climber.extendLeftArm();
      up2 = m_climber.extendRightArm();
    }
    x = 0;
    while(x < 20000) {
      m_climber.extendLeftArmSlow();
      m_climber.extendRightArmSlow();
      x ++;
    }
    m_climber.stopRightArm();
    m_climber.stopLeftArm();
    extend = 4; 
    return;
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
