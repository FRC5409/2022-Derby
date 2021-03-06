// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class kDriveTrain {
        // Motor ports
        // public static final int CANLeftDriveFront = 1;
        // public static final int CANLeftDriveRear = 4;

        // public static final int CANRightDriveFront = 3;
        // public static final int CANRightDriveRear = 2;

        public static final int CANRightDriveFront = 1;
        public static final int CANRightDriveRear = 4;

        public static final int CANLeftDriveFront = 3;
        public static final int CANLeftDriveRear = 2;

        // Drive mode constants
        public static final int ARCADE_DRIVE = 1; // Arcade Drive
        public static final int AADL_DRIVE = 2; // Addl Drive (Arcade drive)
        public static final int C_DRIVE = 3; // Curvature drive
        public static final int T_DRIVE = 4; // Tank Drive

        public static final double UP_SHIFT = 300;
        public static final double DOWN_SHIFT = 260;

        public static final int MOTOR_CURRENT_LIMIT = 80;

        // TODO: Run characterization to get proper gains
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double PDriveVel = 8.5;

        // TODO: Find track width in meters
        public static final double TrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(
                TrackwidthMeters);

        public static final double MaxSpeedMetersPerSecond = 3;
        public static final double MaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double RamseteB = 2;
        public static final double RamseteZeta = 0.7;
    }

    public static class kPneumatics {
        public static final int PCMId = 0;
    }
}
