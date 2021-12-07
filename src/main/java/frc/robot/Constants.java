// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DrivetrainConstants{
         /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase

        public static final int PIGEON_ID = 0; // FIXME Set Pigeon ID

        public static final GearRatio SWERVE_GEAR_RATIO = GearRatio.L2; 
        public static final class FrontLeftSwerveConstants{
            public static final int STEER_MOTOR_ID = 1;
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ENCODER_ID = 3;
            public static final double OFFSET = 0;
        }

        public static final class FrontRightSwerveConstants{
            public static final int STEER_MOTOR_ID = 4;
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ENCODER_ID = 6;
            public static final double OFFSET = 0;
        }

        public static final class BackLeftSwerveConstants{
            public static final int STEER_MOTOR_ID = 7;
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int ENCODER_ID = 9;
            public static final double OFFSET = 0;
        }

        public static final class BackRightSwerveConstants{
            public static final int STEER_MOTOR_ID = 10;
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ENCODER_ID = 12;
            public static final double OFFSET = 0;
        }

        public static final class TurnGains {
            public static final double kP = .2;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kTurnToleranceRad = 0.025;
            public static final double kTurnRateToleranceRadPerS = .17;
        }
    }
}