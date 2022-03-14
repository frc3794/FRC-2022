// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  private final TalonSRX m_leftMotor = new TalonSRX(IndexerConstants.kLeftMotor);
  private final TalonSRX m_rightMotor = new TalonSRX(IndexerConstants.kRightMotor);

  /** Creates a new Indexer. */
  public Indexer() {
    m_rightMotor.follow(m_leftMotor);
    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(true);
  }

  public void rotate() {
    m_leftMotor.set(ControlMode.PercentOutput, 1);
  }

  public void rotateInversed() {
    m_leftMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stop() {
    m_leftMotor.set(ControlMode.PercentOutput, 0);
  }
}
