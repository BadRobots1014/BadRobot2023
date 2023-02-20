// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

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

        public final static int kBalanceButton = 1; //Second joystick

        public final static int kArmHighButton = 3;//left joystick middle top button
        public final static int kArmMediumButton = 5;//left joystick top right button
        public final static int kArmLowButton = 2;//left joystick middle bottom button
        public final static int kArmStoreButton = 4;//left joystick top left button

        public final static int kGrabberManFButton = 6; //right joystick base left top button
        public final static int kGrabberManRButton = 7; //right joystick base left bottom button
        public final static int kGrabberPresetFButton = 11; //right joystick base right top button
        public final static int kGrabberPresetRButton = 10; //right joystick base right bottom button
        
    }
    public final class ArmConstants{
        //set preset arm positions
        public final static int kArmHighPos = 3;
        public final static String kArmHigh = "HIGH";

        public final static int kArmMiddlePos = 2;
        public final static String kArmMedium = "MEDIUM";
        
        public final static int kArmLowPos = 1;
        public final static String kArmLow = "LOW";

        public final static int kArmStoredPos = 0;
        public final static String kArmStored = "STORED";

        // Grabber Running Modes
        public final static String kManualRunForward = "Manual Run Forward";
        public final static String kManualRunBackward = "Manual Run Backward";
        public final static String kPresetRunForward = "Preset Run Forward";
        public final static String kPresetRunBackward = "Preset Run Backward";
        public final static String kBrake = "Brake";


        // Grabber power constants
        public final static double kGrabberPowerF = 0.25;
        public final static double kGrabberPowerR = -0.25;
        public final static double kGrabberBrake = 0;
        

        // motor ports
        public final static int kGrabberPort = 7; 
        public final static int kExtenderPort = 5; 
        public final static int kWinchPort = 6; 

        


    }

    public final class DriveConstants {

        public final static int kRightPort = 2;
        public final static int kLeftPort = 3;

    }

    public final class BlinkinConstants {

        public final static int kBlinkinPort = 0;

    }

    public final static class SensorConstants {

        public final static I2C.Port ColorSensorPort = I2C.Port.kMXP;
        
    }

    public final static class MovementConstants {

        public final static String kStationary = "Stationary";
        public final static String kPivotingOffOfRight = "Pivoting off of right";
        public final static String kPivotingOffOfLeft = "Pivoting off of left";
        public final static String kBackward = "Backward";
        public final static String kTurningCounterclockwise = "Turning Counterclockwise";
        public final static String kTurningClockwise = "Turning Clockwise";
        public final static String kForward = "Forward";
        public final static String kSpinningInPlace = "Spinning in place";
        public final static String kGetDirectionEdgeCase = "getDirection edge case";

    }

    public final static class BlinkinPatternConstants {

        public final static double kSolidRed = 0.61; 
        public final static double kSolidBlue = 0.87; 
        public final static double kSolidWhite = 0.93; 
        public final static double kStrobeRed = -0.11;
        public final static double kStrobeBlue = -0.09;

    }

    public final static class GyroConstants {

        public final static double kBalanceThreshold = 5; //In degrees off of upright
        public final static double kBalanceSpeed = 0.027;

    }

    public final static class EncoderConstants {

        public final static double kDefaultDPP = 4.0/256.0;

        public final static int kExtenderChannelA = 0;
        public final static int kExtenderChannelB = 1;
        public final static double kExtenderMinRate = 10.0;
        public final static boolean kExtenderIsReversed = false;
        public final static int kExtenderSampleSize = 5;

        public final static int kWinchChannelA = 0;
        public final static int kWinchChannelB = 1;
        public final static double kWinchMinRate = 10.0;
        public final static boolean kWinchIsReversed = false;
        public final static int kWinchSampleSize = 5;

    }

}
