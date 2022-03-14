// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.MoveDrivetrain;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
      m_frontRight, m_rearRight);

  private final Encoder m_leftEncoder = new Encoder(DrivetrainConstants.kEncoderPorts[0][0],
      DrivetrainConstants.kEncoderPorts[0][1]);
  private final Encoder m_rightEncoder = new Encoder(DrivetrainConstants.kEncoderPorts[1][0],
      DrivetrainConstants.kEncoderPorts[1][1]);

  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  private final DifferentialDrive m_drive;

  private final AnalogGyro m_gyro = new AnalogGyro(1);
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  private final Field2d m_field = new Field2d();

  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      new Pose2d());

  private final DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDoubleNEOPerSide,
      KitbotGearing.k10p71,
      KitbotWheelSize.kSixInch,
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  private final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(
      DrivetrainConstants.kTrackWidth);

  public Drivetrain() {
    m_leftMotors.setInverted(true);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    m_leftEncoder
        .setDistancePerPulse(2 * Math.PI * DrivetrainConstants.kWheelRadius / DrivetrainConstants.kEncoderResolution);
    m_rightEncoder
        .setDistancePerPulse(2 * Math.PI * DrivetrainConstants.kWheelRadius / DrivetrainConstants.kEncoderResolution);

    this.setDefaultCommand(new MoveDrivetrain(this));

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(m_leftMotors.get() * RobotController.getInputVoltage(),
        m_rightMotors.get() * RobotController.getInputVoltage());

    m_driveSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());

    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());

    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void stop(){
    this.m_drive.arcadeDrive(0, 0);
  }
}
