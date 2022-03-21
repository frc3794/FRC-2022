// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.PneumaticsConstants;
import frc.robot.commands.Auto;
import frc.robot.commands.FeedCargo;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.TransportCargo;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final XboxController m_subsystemController = Robot.getSubsystemController();

  private final Intake m_intake = new Intake();

  private final Compressor m_pcmCompressor = new Compressor(
      PneumaticsConstants.kPCMPort,
      PneumaticsModuleType.CTREPCM);

  private final Shooter m_shooter = new Shooter();

  private final Indexer m_indexer = new Indexer();

  private final Drivetrain m_drivetrain = new Drivetrain();

  private final Climber m_climber = new Climber();

  private final Command superSimpleAuto = superSimpleAutonomous();

  private final Command simpleAuto = null;

  private final Command complexAuto = null;

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_pcmCompressor.enableDigital();

    new JoystickButton(m_subsystemController, Button.kA.value)
        .whileHeld(new FeedCargo(m_intake));

    new JoystickButton(m_subsystemController, Button.kB.value)
        .toggleWhenPressed(new ShootCargo(m_shooter, 4500));

    new JoystickButton(m_subsystemController, Button.kY.value)
        .toggleWhenPressed(new ShootCargo(m_shooter, 2500));

    new JoystickButton(m_subsystemController, Button.kX.value)
        .whileHeld(new TransportCargo(m_indexer));

    m_chooser.setDefaultOption ("Super Simple Autonomous", superSimpleAuto);
    m_chooser.addOption ("Simple Autonomous", simpleAuto);
    m_chooser.addOption ("Complex Autonomous", complexAuto);

    SmartDashboard.putData(":", m_chooser);
  }

  public Command superSimpleAutonomous () {
    ShootCargo shootCommand = new ShootCargo(m_shooter, 4500);
    TransportCargo transportCargo = new TransportCargo(m_indexer);
    Auto auto = new Auto(m_drivetrain);

    return shootCommand.withTimeout(4)
        .deadlineWith(
            new WaitCommand(3).andThen(transportCargo))
        .andThen(auto.withTimeout(2));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
