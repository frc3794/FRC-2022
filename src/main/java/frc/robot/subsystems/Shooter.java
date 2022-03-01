package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.examples.frisbeebot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;


public class Shooter extends PIDSubsystem {
  private final PWMSparkMax m_shooterMotor1 = new PWMSparkMax(ShooterConstants.kShooterMotor1Port);
  private final PWMSparkMax m_shooterMotor2 = new PWMSparkMax(ShooterConstants.kShooterMotor2Port);
  private final PWMTalonSRX m_feederMotor = new PWMTalonSRX(ShooterConstants.kFeederMotorPort);
  
  private final Encoder m_shooter1Encoder =
      new Encoder(
          ShooterConstants.kEncoderPorts[0],
          ShooterConstants.kEncoderPorts[1],
          ShooterConstants.kEncoderReversed);

  private final Encoder m_shooter2Encoder =
    new Encoder(
        ShooterConstants.kEncoderPorts[0],
        ShooterConstants.kEncoderPorts[1],
        ShooterConstants.kEncoderReversed);

  private final Encoder m_feederEncoder =
  new Encoder(
      ShooterConstants.kEncoderPorts[0],
      ShooterConstants.kEncoderPorts[1],
      ShooterConstants.kEncoderReversed);

  private final SimpleMotorFeedforward m_shooter1Feedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

  private final SimpleMotorFeedforward m_shooter2Feedforward =
  new SimpleMotorFeedforward(
      ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

  private final SimpleMotorFeedforward m_feederFeedforward =
  new SimpleMotorFeedforward(
      ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

  /** The shooter subsystem for the robot. */
  public Shooter() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
    m_shooter1Encoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    m_shooter2Encoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    m_feederEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    setSetpoint(/**ShooterConstants.kShooterTargetRPS*/);
  }


  @Override
  public void useOutput(double output, double setpoint) {

    m_shooterMotor1.setVoltage(output + m_shooter1Feedforward.calculate(setpoint));
    m_shooterMotor2.setVoltage(output + m_shooter2Feedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_shooter1Encoder.getRate();
    return m_shooter2Encoder.getRate();
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

/*
PWMTalonSRX
2.004ms = full "forward"
1.520ms = the "high end" of the deadband range
1.500ms = center of the deadband range (off)
1.480ms = the "low end" of the deadband range
0.997ms = full "reverse"

PWMSparkMax
2.003ms = full "forward"
1.550ms = the "high end" of the deadband range
1.500ms = center of the deadband range (off)
1.460ms = the "low end" of the deadband range
0.999ms = full "reverse"
*/
