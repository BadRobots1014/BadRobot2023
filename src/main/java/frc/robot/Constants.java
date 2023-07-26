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

    public final class ControllerConstants {

        public final static int kRightJoystickPort = 0;
        public final static int kLeftJoystickPort = 1;

        public final static double kMaxThrottle = 1.0;
        public final static double kSlowThrottle = 0.5;
        
        public final static int kThrottleButton = 2;

        public final static int kSpinUpButton = 1;
        public final static int kShootButton = 1; //Note that this is on the left joystick.
        public final static int kShootBackButton = 3;
        
    }

    public final class DriveConstants {

        public final static int kRightPort = 13;
        public final static int kSecondRightPort = 14;
        public final static int kLeftPort = 10;
        public final static int kSecondLeftPort = 11;

    }

    public final class ShooterConstants {

        public final static int kFrontPort = 12;
        public final static int kBackPort = 21;
        public final static int kFlipperPort = 22;
        public final static double kFlipperPower = .3;
        public final static double kFlipperBackPower = -.3;
        public final static int kWinchPort = 23;

    }

    public final class BlinkinConstants {
        
        public final static int kBlinkinPort = 0;
        
    }

}
