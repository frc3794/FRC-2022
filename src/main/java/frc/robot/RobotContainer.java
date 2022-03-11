// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.commands.ForwardCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();

  private final ForwardCommand m_autoCommand = new ForwardCommand(m_drivetrain);

  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_subsystemController = new XboxController(1);

  private final Intake m_intake = new Intake();

  private final Compressor m_pcmCompressor = new Compressor(
      PneumaticsConstants.pcmPort,
      PneumaticsModuleType.CTREPCM);

  private final Shooter m_shooter = new Shooter();

  public RobotContainer() {

    m_pcmCompressor.enableDigital();

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(m_subsystemController, Button.kA.value)
        .whenPressed(new InstantCommand(m_intake::toggle, m_intake));

    new JoystickButton(m_subsystemController, Button.kY.value)
        .whenPressed(new InstantCommand(m_shooter::enable, m_shooter));

    new JoystickButton(m_subsystemController, Button.kB.value)
        .whenPressed(new InstantCommand(m_shooter::disable, m_shooter));

    new JoystickButton(m_subsystemController, Button.kX.value)
        .whileHeld(new StartEndCommand(m_shooter::runFeeder, m_shooter::stopFeeder,
            m_shooter));
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
    rot *= DrivetrainConstants.kSensibilityPercent * -0.8;

    m_drivetrain.arcadeDrive(fwd, rot);
  }
}
