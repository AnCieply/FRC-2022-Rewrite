// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class Drive {
        // CAN ids
        public static final int kFrontLeftMotorID = 3;
        public static final int kBackLeftMotorID = 4;
        public static final int kFrontRightMotorID = 1;
        public static final int kBackRightMotorID = 2;

        // Direction constants (NEW)
        public static final boolean kLeftInverted = false;
        public static final boolean kRightInverted = true;
        public static final boolean kGyroReversed = true;
        public static final boolean kRotationReversed = false;

        // Physical robot properties (NEW)
        public static final double kTrackWidth = 22;
        public static final double kWheelRadius = 3;
        public static final double kGearRatio = 10.71;
        public static final double kEncoderResolution = 2048;

        // Feed forward values.
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        // PID values.
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }
}
