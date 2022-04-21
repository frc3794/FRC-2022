// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticsConstants;

public class Intake extends SubsystemBase {

  private final Solenoid m_cylinderLeft = new Solenoid(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM,
      IntakeConstants.kCylinderLeft);

  private final Solenoid m_cylinderRight = new Solenoid(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM,
      IntakeConstants.kCylinderRight);

  private final TalonSRX m_getter = new TalonSRX(IntakeConstants.kMotor);
  //private final CANSparkMax m_getter = new CANSparkMax(IntakeConstants.kMotor, MotorType.kBrushless);

  public Intake() {
    m_getter.setInverted(true);
  }

  public void open() {
    m_cylinderLeft.set(true);
    m_cylinderRight.set(true);
    m_getter.set(ControlMode.PercentOutput, 0.9);
  }

  public void close() {
    m_cylinderLeft.set(false);
    m_cylinderRight.set(false);
    m_getter.set(ControlMode.PercentOutput, 0);
  }
}
