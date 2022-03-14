// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.commands.Climb;

public class Climber extends SubsystemBase {

  private final VictorSPX m_leftMotor = new VictorSPX(ClimberConstants.kLeftMotor);
  private final VictorSPX m_rightMotor = new VictorSPX(ClimberConstants.kRightMotor);

  private final Solenoid m_leftCylinder = new Solenoid(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM,
      ClimberConstants.kLeftCylinder);

  private final Solenoid m_rightCylinder = new Solenoid(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM,
      ClimberConstants.kRightCylinder);

  public Climber() {
    setDefaultCommand(new Climb(this));
    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void extendLeftArm() {
    m_leftMotor.set(ControlMode.PercentOutput, -1);
  }

  public void extendRightArm() {
    m_rightMotor.set(ControlMode.PercentOutput, 1);
  }

  public void contractLeftArm() {
    m_leftMotor.set(ControlMode.PercentOutput, 0.4);
  }

  public void contractRightArm() {
    m_rightMotor.set(ControlMode.PercentOutput, -0.4);
  }

  public void stopLeftArm() {
    m_leftMotor.set(ControlMode.PercentOutput, 0);
  }

  public void stopRightArm() {
    m_rightMotor.set(ControlMode.PercentOutput, 0);
  }

  public void slideOut() {
    m_leftCylinder.set(true);
    m_rightCylinder.set(true);
  }

  public void slideIn() {
    m_leftCylinder.set(false);
    m_rightCylinder.set(false);
  }
}
