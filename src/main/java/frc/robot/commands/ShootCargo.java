package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootCargo extends CommandBase {

  private final Shooter m_shooter;

  public ShootCargo(Shooter shooter) {
    this.m_shooter = shooter;
    addRequirements(this.m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.runFeeder();
    m_shooter.run();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_shooter.stopFeeder();
  }
}
