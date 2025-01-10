// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double stickDeadband = 0.1;

    public static final class DriveTrain {
        public static final int kLeftMasterId = 4;
        public static final int kLeftFollowerId = 3;
        public static final int kRightMasterId = 1;
        public static final int kRightFollowerId = 2;


        public static final InvertType kLeftMasterInversion = InvertType.None;
        public static final InvertType kLeftFollowerInversion = InvertType.FollowMaster;
        public static final InvertType kRightMasterInversion = InvertType.InvertMotorOutput;
        public static final InvertType kRightFollowerInversion = InvertType.FollowMaster;

        public static final double kP = 0.035;
        public static final double kD = 0.0;
        public static final double kI = 0.02;
        public static final double kS = 0.2;
    }

    public static final class vision {
        public static final double aprilFilterTimeConstant = 0.1;
        public static final double aprilFilterPeriod = 0.1;
    }
        
}
