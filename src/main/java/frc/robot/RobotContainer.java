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

  private final Climber m_climber = new Climber();

  private final Indexer m_indexer = new Indexer();

  private final Drivetrain m_drivetrain = new Drivetrain();

  private final int highShoot = 3794;

  private final int lowShoot = 1850;

  private final Timer timer = new Timer();
  private final Timer tm = new Timer();

  public RobotContainer() {
    m_pcmCompressor.enableDigital();

    new JoystickButton(m_subsystemController, Button.kA.value)
        .whileHeld(new FeedCargo(m_intake));

    new JoystickButton(m_subsystemController, Button.kB.value)
        .toggleWhenPressed(new ShootCargo(m_shooter, highShoot));

    new JoystickButton(m_subsystemController, Button.kY.value)
        .toggleWhenPressed(new ShootCargo(m_shooter, lowShoot));

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
    } else if (!buttonValue &&!buttonValue1 && buttonValue2) { 
        return level3();
    }

    return level1 ();
  }

  public Command level1 () {
    timer.reset();
    timer.start();

    while (timer.get() <= 3) {
        m_shooter.runFeeder();
        m_shooter.run(highShoot);
        if (timer.get() >= 0.7) {
            m_indexer.rotate();
        }
        if (tm.get () >= 14.9) {return stopAuto ();}
    }

    m_shooter.stop();
    m_shooter.stopFeeder();
    m_indexer.stop();

    timer.reset();
    timer.start();

    while (timer.get() <= 9) {
    }

    m_drivetrain.moveToDistance(-2, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}
    
    return new WaitCommand(0.1);
  }

  public Command level2 () {
    
    m_intake.open();

    m_drivetrain.moveToDistance(-1, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}
    m_drivetrain.moveToDistance(2.1, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    timer.reset();
    timer.start();

    while (timer.get() <= 2) {
        m_shooter.runFeeder();
        m_shooter.run(highShoot);
        if (timer.get() >= 0.3) {
            m_intake.close();
        }
        if (timer.get() >= 0.7) {
            m_indexer.rotate();
        }
        if (tm.get () >= 14.9) {return stopAuto ();}
    }

    m_shooter.stop();
    m_shooter.stopFeeder();
    m_indexer.stop();

    m_drivetrain.moveToDistance(-2.5, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    m_intake.open();

    m_drivetrain.rotateToAngle(90, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    m_drivetrain.moveToDistance(-1, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    m_intake.close();

    m_drivetrain.rotateToAngle(30, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    m_drivetrain.moveToDistance(3, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    timer.stop();

    timer.reset();
    timer.start();

    while (timer.get() <= 2) {
        m_shooter.runFeeder();
        m_shooter.run(lowShoot);
        if (timer.get() >= 0.7) {
            m_indexer.rotate();
        }
        if (tm.get () >= 14.9) {return stopAuto ();}
    }

    timer.stop();

    m_shooter.stop();
    m_shooter.stopFeeder();
    m_indexer.stop();

    m_drivetrain.stop();

    return new WaitCommand(0.1);
  }

  public Command level3 () {
    tm.reset();
    tm.start();
    
    m_intake.open();

    timer.reset();
    timer.start();

    while (timer.get() <= 1.5) {
        m_shooter.runFeeder();
        m_shooter.run(highShoot);
        if (timer.get() >= 0.6) {
            m_indexer.rotate();
        }
        if (tm.get () >= 14.9) {return stopAuto ();}
    }
    m_shooter.stop();
    m_shooter.stopFeeder();
    m_indexer.stop();

    m_drivetrain.moveToCurve(-2, -2.8, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    m_drivetrain.rotateToAngle(105, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    m_drivetrain.moveToDistance(-3, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    m_drivetrain.moveToCurve(2.5, 1.5, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    m_shooter.runFeeder();
    m_shooter.run(highShoot);

    m_drivetrain.moveToDistance(1, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    timer.reset();
    timer.start();

    while (timer.get() <= 0.9) {
        m_intake.close();
        m_indexer.rotate();
        if (tm.get () >= 14.9) {return stopAuto ();}
    }

    m_shooter.stop();
    m_shooter.stopFeeder();
    m_indexer.stop();

    m_drivetrain.moveToDistance(-4, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}

    m_drivetrain.rotateToAngle(30, tm.get());
    if (tm.get () >= 14.9) {return stopAuto ();}
    
    return new WaitCommand(0.1);
  }

  public Command stopAuto () {
    m_drivetrain.stop();
    m_shooter.stop();
    m_shooter.stopFeeder();
    m_indexer.stop();
    m_intake.close();
    timer.stop();

    return null;
  }
}
 