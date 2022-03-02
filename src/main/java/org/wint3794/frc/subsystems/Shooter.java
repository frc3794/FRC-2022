// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wint3794.frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.wint3794.frc.Constants.ShooterConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {

  private final CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotorFollower = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);

  private final CANSparkMax m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorPort, MotorType.kBrushless);

  private final Encoder m_shooterEncoder = new Encoder(
      ShooterConstants.kEncoderPorts[0],
      ShooterConstants.kEncoderPorts[1],
      ShooterConstants.kEncoderReversed);

  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

  public Shooter() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    m_shooterMotorFollower.follow(m_shooterMotor);
    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    setSetpoint(ShooterConstants.kShooterTargetRPS);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_shooterEncoder.getRate();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void runFeeder() {
    m_feederMotor.set(ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.set(0);
  }
}
