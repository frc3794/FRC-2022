// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.MoveDrivetrain;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import java.lang.Math;
import frc.robot.Constants.PIDAutoConstants;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax m_frontLeft = new CANSparkMax(DrivetrainConstants.kMotorPorts[0],
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rearLeft = new CANSparkMax(DrivetrainConstants.kMotorPorts[1],
      CANSparkMaxLowLevel.MotorType.kBrushless);

  private final CANSparkMax m_frontRight = new CANSparkMax(DrivetrainConstants.kMotorPorts[2],
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rearRight = new CANSparkMax(DrivetrainConstants.kMotorPorts[3],
      CANSparkMaxLowLevel.MotorType.kBrushless);

  private MotorControllerGroup m_leftMotors = new MotorControllerGroup(
      m_frontLeft, m_rearLeft);

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
      m_frontRight, m_rearRight);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private float toAngle = 0.00F;

  private final DifferentialDrive m_drive;

  private final DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDoubleNEOPerSide,
      KitbotGearing.k10p71,
      KitbotWheelSize.kSixInch,
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  private final Timer timer = new Timer();

  private int addAngle;

    /*private double previousEncoderLeft = getEncoderDistance(0);
    private double previousEncoderRight = getEncoderDistance(1);
    private double errorLeft = 100000;
    private double errorRight = 100000;
    private double previousErrorLeft = 0;
    private double previousErrorRight = 0;
    private double errorLeftSum = 0;
    private double errorRightSum = 0;*/

  public Drivetrain() {
    m_leftMotors.setInverted(true);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    /*m_leftEncoder
        .setDistancePerPulse(2 * Math.PI * DrivetrainConstants.kWheelRadius / DrivetrainConstants.kEncoderResolution);
    m_rightEncoder
        .setDistancePerPulse(2 * Math.PI * DrivetrainConstants.kWheelRadius / DrivetrainConstants.kEncoderResolution);*/

    this.setDefaultCommand(new MoveDrivetrain(this));
  }

  @Override
  public void periodic() {
    /*m_odometry.update(m_gyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());*/
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  private double getEncoderDistance (double side) {
    if (side == 0) {
      m_leftEncoder = m_frontLeft.getEncoder();
      return (.1524*Math.PI) * (m_leftEncoder.getPosition() / 10.71);
    } else if (side == 1) {
      m_rightEncoder = m_frontRight.getEncoder();
      return (.1524*Math.PI) * (m_rightEncoder.getPosition() / 10.71);
    } else {
      return 0;
    }
  }

  public void moveToDistance (double setPoint, double tm) {
    double previousEncoderLeft = getEncoderDistance(0);
    double previousEncoderRight = getEncoderDistance(1);
    double errorLeft = 100000;
    double errorRight = 100000;

    timer.reset();
    timer.start();

    while (Math.abs(errorLeft) > 0.35 && Math.abs(errorRight) > 0.35) {
      errorLeft = setPoint + getEncoderDistance(0) - previousEncoderLeft;
      errorRight = setPoint - getEncoderDistance(1) + previousEncoderRight;
      tankDriveVolts(errorLeft * PIDAutoConstants.kP, errorRight * PIDAutoConstants.kP);
      if (timer.get() + tm >= 14.5) {
        break;
      }
    }

  }

  public void rotateToAngle (int angle, double tm) {
    timer.reset();
    timer.start();

    m_gyro.reset();
    double kp = 0.0085;
    if (angle > 0) {
      double error = 1000;
      while (Math.abs(error) > 2) {
        error = angle - m_gyro.getAngle();
        m_rightMotors.set(error*kp);
        m_leftMotors.set(-error*kp);
        if (timer.get() + tm >= 14.6) {break;};
      }
    } else if (angle < 0) {
      double error = 1000;
      while (Math.abs(error) > 2) {
        error = angle - m_gyro.getAngle();
        m_rightMotors.set(-error*kp);
        m_leftMotors.set(error*kp);
        if (timer.get() + tm >= 14.6) {break;};
      }
    }
  }

  public void moveToCurve (double setPointLeft, double setPointRight, double tm) {
    double previousEncoderLeft = getEncoderDistance(0);
    double previousEncoderRight = getEncoderDistance(1);
    double errorLeft = 100000;
    double errorRight = 100000;

    timer.reset();
    timer.start();

    while (Math.abs(errorLeft) > 0.1 && Math.abs(errorRight) > 0.1) {
      errorLeft = setPointLeft + getEncoderDistance(0) - previousEncoderLeft;
      errorRight = setPointRight - getEncoderDistance(1) + previousEncoderRight;
      tankDriveVolts(errorLeft * PIDAutoConstants.kP, errorRight * PIDAutoConstants.kP);
      if (timer.get() + tm >= 14.5) {
        break;
      }
    }
  }

  public boolean rightAngle () {
    return m_gyro.getRoll() >= toAngle ? true : false;
  }

  public void setAngle (int angle) {
    this.addAngle = angle;
  }


  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void tankDriveVoltsLeft(double leftVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_drive.feed();
  }

  public void tankDriveVoltsRight(double rightVolts) {
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void stop(){
    this.m_drive.arcadeDrive(0, 0);
  }
}
