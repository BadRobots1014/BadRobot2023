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

        public final static double kDeadZoneRadius = .1;

        public final static double kMaxThrottle = 1.0;
        public final static double kSlowThrottle = 0.5;
        
        public final static int kThrottleButton = 2;

        public final static int kLineUpButton = 3;
        
    }

    public final class LimelightConstants {

        public final static double kLineUpMaxSpeed = .5;
    }

    public final class DriveConstants {

        public final static int kRightPort = 2;
        public final static int kLeftPort = 3;

    }

    public final class BlinkinConstants {

        public final static int kBlinkinPort = 0;

    }

    public final static class MovementConstants {
        public final static String kStationary = "Stationary";
        public final static String kForward = "Forward";
        public final static String kBackward = "Backward";
        public final static String kTurningCounterclockwise = "Turning Counterclockwise";
        public final static String kTurningClockwise = "Turning Clockwise";
        public final static String kSpinningInPlace = "Spinning in place";
        public final static String kGetDirectionEdgeCase = "getDirection edge case";
    }

    public final static class BlinkinPatternConstants {
        public final static double solidRed = 0.61; 
        public final static double solidBlue = 0.87; 
        public final static double solidGreen = 0.77;
        public final static double solidOrange = 0.65;
        public final static double breatheColor1 = 0.09; //Color1 and 2 have to be physically set. Color1 is green
        public final static double breatheColor2 = 0.29; //Color2 is orange
        public final static double solidWhite = 0.93; 
        public final static double breatheRed = -0.17;
        public final static double breatheBlue = -0.15;
        public final static double confetti = -0.87;
        public final static double solidBlack = 0.99; // Primarily For Errors, when something goes boom with the lights
        

    }

}
