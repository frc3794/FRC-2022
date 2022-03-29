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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
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

  private final String[] strArray = {"Level 1 Autonomous", "Level 2 Autonomous"};

  private final Timer timer = new Timer();

  public RobotContainer() {
    m_pcmCompressor.enableDigital();

    new JoystickButton(m_subsystemController, Button.kA.value)
        .whileHeld(new FeedCargo(m_intake));

    new JoystickButton(m_subsystemController, Button.kB.value)
        .toggleWhenPressed(new ShootCargo(m_shooter, 4500));

    new JoystickButton(m_subsystemController, Button.kY.value)
        .toggleWhenPressed(new ShootCargo(m_shooter, 1850));

    new JoystickButton(m_subsystemController, Button.kX.value)
        .whileHeld(new TransportCargo(m_indexer));

  }

  public Command getAutonomousCommand() {
    boolean buttonValue = SmartDashboard.getBoolean("DB/Button 0", false);
    boolean buttonValue1 = SmartDashboard.getBoolean("DB/Button 1", false);
    boolean buttonValue2 = SmartDashboard.getBoolean("DB/Button 2", false);

    if (buttonValue && !buttonValue1 && !buttonValue2) {
        return level1();
    } else if (!buttonValue && buttonValue1 && !buttonValue2) {
        return level2();
    } else if (!buttonValue && !buttonValue1 && buttonValue2) {
        return level3();
    }

    return level1 ();
  }

public Command level1 () {
    ShootCargo shootCommand = new ShootCargo(m_shooter, 4500);
    TransportCargo transportCargo = new TransportCargo(m_indexer);
    Auto auto = new Auto(m_drivetrain);

    return shootCommand.withTimeout(4)
        .deadlineWith(
            new WaitCommand(3).andThen(transportCargo))
        .andThen(auto.withTimeout(2));
  }

  public Command level2 () {
    ShootCargo shootCommand = new ShootCargo(m_shooter, 4500);
    TransportCargo transportCargo = new TransportCargo(m_indexer);
    Auto auto = new Auto (m_drivetrain);
    
    m_intake.open();
    timer.reset ();
    timer.start();
    while(timer.get() < 1);
    m_drivetrain.moveToDistance(-3.5);
    m_intake.close();
    m_drivetrain.moveToDistance(3);
    timer.stop();

    return shootCommand.withTimeout(2)
        .deadlineWith(
            new WaitCommand(1).andThen(transportCargo))
        .andThen(auto.withTimeout (2));  
    
  }

  public Command level3 () {
    Command x = level2();

    ShootCargo shootCommand = new ShootCargo(m_shooter, 4500);
    TransportCargo transportCargo = new TransportCargo(m_indexer);
    Auto auto = new Auto (m_drivetrain);

    m_drivetrain.rotateToAngle(-90);

    m_drivetrain.moveToDistance(1);

    m_drivetrain.rotateToAngle(-90);

    return shootCommand.withTimeout(2)
        .deadlineWith(
            new WaitCommand(1).andThen(transportCargo))
        .andThen(auto.withTimeout (2)); 
  }

}
