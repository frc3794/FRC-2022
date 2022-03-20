// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_pidController_2;
  private RelativeEncoder m_encoder_2;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private final CANSparkMax m_shooterMotor_1 = new CANSparkMax(ShooterConstants.kShooterMotorPort_1, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotor_2 = new CANSparkMax(ShooterConstants.kShooterMotorPort_2, MotorType.kBrushless);

  private final TalonSRX m_feederMotor = new TalonSRX(ShooterConstants.kFeederMotorPort);

  public Shooter() {
    m_shooterMotor_1.setInverted(true);
    m_shooterMotor_2.setInverted(false);
    
    m_pidController = m_shooterMotor_1.getPIDController();
    m_encoder = m_shooterMotor_1.getEncoder();

    m_pidController_2 = m_shooterMotor_2.getPIDController();
    m_encoder_2 = m_shooterMotor_2.getEncoder();

    // PID coefficients
    kP = 0.000055; 
    kI = 0.00000058;
    kD = 0.00000048; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = 0;
    maxRPM = 5500;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController_2.setP(kP);
    m_pidController_2.setI(kI);
    m_pidController_2.setD(kD);
    m_pidController_2.setIZone(kIz);
    m_pidController_2.setFF(kFF);
    m_pidController_2.setOutputRange(kMinOutput, kMaxOutput);
    }

  public void runFeeder() {
    m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void run(double setPoint) {
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    m_pidController_2.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void stop() {
    m_shooterMotor_1.set(0);
    m_shooterMotor_2.set(0);
  }
}
