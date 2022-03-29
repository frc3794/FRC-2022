// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Shooter extends SubsystemBase {

  private final CANSparkMax m_shooterMotor_1 = new CANSparkMax(
      ShooterConstants.kShooterMotorPorts[0], MotorType.kBrushless);

  private final CANSparkMax m_shooterMotor_2 = new CANSparkMax(
      ShooterConstants.kShooterMotorPorts[1], MotorType.kBrushless);

  private final TalonSRX m_feederMotor = new TalonSRX(ShooterConstants.kFeederMotorPort);
  
  private final DigitalOutput R = new DigitalOutput(4);

  public Shooter() {
    m_shooterMotor_1.setInverted(ShooterConstants.kShooterMotorsInverted[0]);
    m_shooterMotor_2.setInverted(ShooterConstants.kShooterMotorsInverted[1]);

    configurePIDController(m_shooterMotor_1.getPIDController());
    configurePIDController(m_shooterMotor_2.getPIDController());
  }

  public void runFeeder() {
    m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void run(double setPoint) {
    m_shooterMotor_1.getPIDController().setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    m_shooterMotor_2.getPIDController().setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void stop() {
    m_shooterMotor_1.set(0);
    m_shooterMotor_2.set(0);
  }

  private void configurePIDController(SparkMaxPIDController pidController) {
    pidController.setP(ShooterConstants.kP);
    pidController.setI(ShooterConstants.kI);
    pidController.setD(ShooterConstants.kD);
    pidController.setIZone(ShooterConstants.kIz);
    pidController.setFF(ShooterConstants.kFF);
    pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
  }

  public void colorLeds (int b) {
    boolean x = false;
    if (b >= 1) {x = true;} else {x = false;}
    R.set(x);
  } 
}
