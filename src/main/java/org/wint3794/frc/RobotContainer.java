// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wint3794.frc;

import org.wint3794.frc.commands.BackwardCommand;
import org.wint3794.frc.commands.ForwardCommand;
import org.wint3794.frc.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();

  private final ForwardCommand m_autoCommand = new ForwardCommand(m_drivetrain);

  private final XboxController m_controller = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(m_controller, XboxController.Button.kA.value).whileHeld(new ForwardCommand(m_drivetrain));
    new JoystickButton(m_controller, XboxController.Button.kB.value).whileHeld(new BackwardCommand(m_drivetrain));
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
