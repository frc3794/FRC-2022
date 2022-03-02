// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    var frontleft = new CANSparkMax(0, MotorType.kBrushless);
    var backleft = new CANSparkMax(1, MotorType.kBrushless);
    var leftMotors = new MotorControllerGroup(frontleft, backleft);

    var frontright = new CANSparkMax(2, MotorType.kBrushless);
    var backright = new CANSparkMax(3, MotorType.kBrushless);
    var rightMotors = new MotorControllerGroup(frontright, backright);

    var drive = new DifferentialDrive(leftMotors, rightMotors);

    var leftencoder = new Encoder (0, 1);
    var rightencoder = new Encoder (1, 2);
    var horencoder = new Encoder (2, 3);
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
