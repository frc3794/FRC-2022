// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class DrivetrainConstants {
        public static final double kEncoderResolution = 1024;

        public static final double kWheelRadius = Units.inchesToMeters(3);
        public static final double kTrackWidth = Units.inchesToMeters(27);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kSensibilityPercent = 0.8;

        public static final int[] kMotorPorts = { 1, 4, 2, 3 }; // FL, RL, FR, RR

        public static final int[][] kEncoderPorts = { { 0, 1 }, { 2, 3 } };
    }

    public static class ShooterConstants {
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final int[] kShooterMotorPort = { 5, 6 };
        public static final int kFeederMotorPort = 10;

        public static final int[] kEncoderPorts = { 6, 7 }; // Only Shooter Ports
        public static final boolean kEncoderReversed = false;

        public static final double kFeederSpeed = 1;

        public static final double kSVolts = 0;
        public static final double kVVoltSecondsPerRotation = 0;
        public static final double kShooterTargetRPS = 0;
        public static final double kEncoderDistancePerPulse = 0;
        public static final double kShooterToleranceRPS = 0;
    }

}
