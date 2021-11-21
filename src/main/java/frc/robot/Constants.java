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
        public static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig(1, 2, 3, 0, GearRatio.L2);
        public static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig(4, 5, 6, 0, GearRatio.L2);
        public static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig(7, 8, 9, 0, GearRatio.L2);
        public static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig(10, 11, 12, 0, GearRatio.L2);
        }
}