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
    
    public static final class ClimberConstants {

        //Ports for climbers
        public static final int kLeftClimberPort = 35;
        public static final int kRightClimberPort = 34;

        //Climber movement speed
        public static final double climberUpSpeed = .3;
        /*Code here*/
    }

    public static final class ShooterConstants {
        public static final int kShooterPort = 7;

        //Make inversions here for forward/backward
        //Positive = forward or toward gatherer
        //Negative = backward or away from gatherer
        public static final double farShotPower = 0.6;
        public static final double farBackShotPower = -0.6;
        public static final double closeShotPower = 0.55;
        public static final double closeBackShotPower = -0.5;
    }

    public final class IndexerConstants {
        public static final int kLowerIndexerSpeedController = 29;
        public static final int kUpperIndexerSpeedController = 15;

        public static final int kLowerIndexerSensor = 10;
        public static final int KUpperIndexerSensor = 12;

        public static final int kIndexerMaxSpeed = 1;

        public static final double kIndexTime = 0.51;
    }

}
