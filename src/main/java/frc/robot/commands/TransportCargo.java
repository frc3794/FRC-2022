package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class TransportCargo extends CommandBase {

  private final Indexer m_indexer;
  
  public TransportCargo(Indexer indexer) {
    this.m_indexer = indexer;
    addRequirements(this.m_indexer);
  }

  @Override
  public void initialize() {
    m_indexer.rotate();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
