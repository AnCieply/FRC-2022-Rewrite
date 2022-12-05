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
    public final static class Controller {
        // USB port to plug controller into.
        public static final int kPort = 0;
    }

    public final static class Drive {
        // CAN ids.
        public static final int kFrontLeftMotorID = 3;
        public static final int kBackLeftMotorID = 4;
        public static final int kFrontRightMotorID = 1;
        public static final int kBackRightMotorID = 2;

        // Direction constants.
        public static final boolean kLeftInverted = false;
        public static final boolean kRightInverted = true;
        public static final boolean kGyroReversed = true;
        public static final boolean kRotationReversed = false;

        // Physical robot properties.
        public static final double kTrackWidth = 22;
        public static final double kWheelRadius = 3;
        public static final double kGearRatio = 10.71;
        public static final double kEncoderResolution = 2048;

        // Ramp val
        public static final double kRampInSec = 0.1875;

        // Feed forward values.
        public static final double kS = 0.59217;
        public static final double kV = 2.4309;
        public static final double kA = 0.37474;

        // PID values.
        public static final double kP = 3.3619;
        public static final double kI = 0;
        public static final double kD = 0;
        
        // Misc characterization values.
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Movement limits.
        public static final double kVelocityMax = 0.9144; // 6 ft/s
        public static final double kAcclerationMax = 0.6096; // 4 ft/s^2
    }

    public static final class ShooterConst {
        // CAN ids.
        public static final int kLeaderMotorID = 5;
        public static final int kFollowerMotorID = 6;
    
        // Default inversion.
        public static final boolean kFrontMotorInvert = true;
    }
}
