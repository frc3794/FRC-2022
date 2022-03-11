// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Intake extends SubsystemBase {

  private final Solenoid m_cylinderLeft = new Solenoid(PneumaticsModuleType.CTREPCM,
      PneumaticsConstants.intakeCylinderLeft);

  private final Solenoid m_cylinderRight = new Solenoid(PneumaticsModuleType.CTREPCM,
      PneumaticsConstants.intakeCylinderRight);

  private boolean isOpen = false;

  /** Creates a new Intake. */
  public Intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void open() {
    m_cylinderLeft.set(true);
    m_cylinderRight.set(true);
    this.isOpen = true;
  }

  public void close() {
    m_cylinderLeft.set(false);
    m_cylinderRight.set(false);
    this.isOpen = false;
  }

  public void toggle() {
    if (this.isOpen) {
      close();
    } else {
      open();
    }

    this.isOpen = !this.isOpen;
  }
}
