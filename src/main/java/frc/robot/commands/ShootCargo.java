package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootCargo extends CommandBase {

  private final Shooter m_shooter;
  private double setPoint = 0;

  public ShootCargo(Shooter shooter, double setPoint) {
    this.m_shooter = shooter;
    addRequirements(this.m_shooter);
    this.setPoint = setPoint;
  }

  @Override
  public void initialize() {
    m_shooter.runFeeder();
    m_shooter.run(this.setPoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFeeder();
    m_shooter.stop();
  }
}
