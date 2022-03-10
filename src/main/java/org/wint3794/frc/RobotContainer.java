// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wint3794.frc;

import org.wint3794.frc.Constants.DrivetrainConstants;
import org.wint3794.frc.commands.ForwardCommand;
import org.wint3794.frc.subsystems.Drivetrain;
import org.wint3794.frc.subsystems.Shooter;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();

  private final ForwardCommand m_autoCommand = new ForwardCommand(m_drivetrain);

  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_subsystemController = new XboxController(1);

  //tmp
  private final Solenoid colgada = new Solenoid(PneumaticsModuleType.CTREPCM, 6);

  private final Shooter m_shooter = new Shooter();

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(m_subsystemController, Button.kA.value)
      .whenPressed(new InstantCommand(m_shooter::enable, m_shooter));

    new JoystickButton(m_subsystemController, Button.kB.value)
      .whenPressed(new InstantCommand(m_shooter::disable, m_shooter));

    new JoystickButton(m_subsystemController, Button.kX.value)
      .whileHeld(new StartEndCommand(m_shooter::runFeeder, m_shooter::stopFeeder, m_shooter));

    new JoystickButton(m_subsystemController, Button.kY.value)
      .whenPressed(new StartEndCommand(
        () -> colgada.set(true), () -> colgada.set(false) ));
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
