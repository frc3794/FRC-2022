// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static class DrivetrainConstants {
        public static final double kEncoderResolution = 1024;

        public static final double kWheelRadius = Units.inchesToMeters(3);
        public static final double kTrackWidth = Units.inchesToMeters(27);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kSensibilityPercent = 0.85;

        public static final int[] kMotorPorts = { 1, 4, 2, 3 }; // FL, RL, FR, RR

        public static final int[][] kEncoderPorts = { { 0, 1 }, { 2, 3 } };
    }

    public static class ShooterConstants {
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final int[] kShooterMotorPorts = { 5, 6 };
        public static final int kFeederMotorPort = 10;

        public static final int[] kEncoderPorts = { 6, 7 }; // Only Shooter Ports
        public static final boolean kEncoderReversed = false;

        public static final double kFeederSpeed = 1;

        public static final double kSVolts = 0;
        public static final double kVVoltSecondsPerRotation = 0;
        public static final double kShooterTargetRPS = 70;
        public static final double kEncoderDistancePerPulse = 10;
        public static final double kShooterToleranceRPS = 10;
    }

    public static class IntakeConstants {
        public static final int kCylinderLeft = 2;
        public static final int kCylinderRight = 3;

        public static final int kMotor = 9;
    }

    public static class IndexerConstants {
        public static final int kLeftMotor = 8;
        public static final int kRightMotor = 12;
    }

    public static class PneumaticsConstants {
        public static final int kPCMPort = 16;
    }

    public static class ClimberConstants {
        public static final int kLeftMotor = 15;
        public static final int kRightMotor = 14;

        public static final int kLeftCylinder = 6;
        public static final int kRightCylinder = 7;

        public static final int kLeftUpperLimit = 0;
        public static final int kLeftLowerLimit = 2;

        public static final int kRightUpperLimit = 1;
        public static final int kRightLowerLimit = 3;
    }
}
