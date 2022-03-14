// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class FeedCargo extends CommandBase {

  private final Intake m_intake;

  public FeedCargo(Intake intake) {
    this.m_intake = intake;
    addRequirements(this.m_intake);
  }

  @Override
  public void initialize() {
    m_intake.open();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.close();
  }
}
