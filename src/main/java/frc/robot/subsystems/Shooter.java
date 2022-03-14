// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushed);

  private final TalonSRX m_feederMotor = new TalonSRX(ShooterConstants.kFeederMotorPort);

  public Shooter() {
    m_shooterMotor.setInverted(true);
  }

  public void runFeeder() {
    m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void run() {
    m_shooterMotor.set(0.9);
  }

  public void stop() {
    m_shooterMotor.set(0);
  }
}
