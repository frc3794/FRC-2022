// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkMax;

public class DriveTrain extends SubsystemBase {
   
  private CANSparkMax frontleft;
  private CANSparkMax backleft;
  private MotorControllerGroup leftMotors;
  private CANSparkMax frontright;
  private CANSparkMax backright;
  private MotorControllerGroup rightMotors;
  private DifferentialDrive drive;
  private Encoder leftencoder; 
  private Encoder rightencoder;
  private Encoder horencoder;


  public DriveTrain() {

    frontleft = new CANSparkMax(0);
    backleft = new CANSparkMax(1);
    leftMotors = new MotorControllerGroup(frontleft, backleft);

    frontright = new CANSparkMax(2);
    backright = new CANSparkMax(3);
    rightMotors = new MotorControllerGroup(frontright, backright);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    leftencoder = new Encoder (0);
    rightencoder = new Encoder (1);
    horencoder = new Encoder (2);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
