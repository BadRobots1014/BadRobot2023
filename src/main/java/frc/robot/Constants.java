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

        // For Joystick
        public final static int kRightJoystickPort = 0;
        public final static int kLeftJoystickPort = 1;

        public final static double kDeadZoneRadius = .1;

        public final static double kMaxThrottle = 1.0;
        public final static double kSlowThrottle = 0.5;
        
        public final static int kThrottleButton = 2;
        public final static int kBalanceButton = 1; //Second joystick

        // public final static int kArmHighButton = 3;//left joystick middle top button
        // public final static int kArmMediumButton = 5;//left joystick top right button
        // public final static int kArmLowButton = 2;//left joystick middle bottom button
        // public final static int kArmStoreButton = 4;//left joystick top left button
        // public final static int kDunkTrigger = 1;//left trigger joystick
        public final static int kArmMoveUp = 11;
        public final static int kArmMoveDown = 10;
        public final static int kArmZeroButton = 7;

        // public final static int kGrabberFButton = 3; //right joystick middle top button
        // public final static int kGrabberRButton = 2; //right joystick middle bottom button

        public final static int kDriveStraightButton = 1;//Right joystick trigger
        
        // For Xbox
        public final static int kXboxControllerPort = 2;
        public final static double kXboxDeadZoneRadius = .1;

        //Xbox buttons in RobotContainer
        
    }
    public final class ArmConstants{

        //Max + min positions
        public final static double kMaxHeight = 35;
        public final static double kMinHeight = 0;

        //set preset arm positions
        public final static double kArmHighPos = 27;
        public final static String kArmHigh = "HIGH";

        public final static double kArmMediumPos = 16;
        public final static String kArmMedium = "MEDIUM";
        
        public final static double kArmLowPos = 5;
        public final static String kArmLow = "LOW";

        public final static double kArmStoredPos = 0;
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

        // Grabber State Constants
        public final static double kGrabberAmpMax = 35; //Approximately how min number of amps motor reads out when it fills
        public final static String kGrabberFilled = "Grabber is Filled";
        public final static String kGrabberEmpty = "Grabber is Empty";

        // motor ports
        public final static int kGrabberPort = 7; 
        public final static int kExtenderPort = 5; 
        public final static int kWinchPort = 6;

        //Winch
        public final static double kWinchUpSpeed = .3;
        public final static double kWinchDownSpeed = -.5;

    }

    public final class DriveConstants {

        public final static int kRightAPort = 1;
        public final static int kRightBPort = 2;
        public final static int kLeftAPort = 3;
        public final static int kLeftBPort = 4;

    }

    public final class BlinkinConstants {

        public final static int kBlinkinPort = 0;

    }

    public final static class SensorConstants {


        public final static I2C.Port kColorSensorPort = I2C.Port.kMXP;
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
        public final static double blinkingRed = -0.25; //Backward on the driveStraightCommand
        public final static double blinkingBlue = -0.23; //Forward on the driveStraightCommand
        public final static double solidBlack = 0.99; // Primarily For Errors, when something goes boom with the lights
    }

    public final static class GyroConstants {

        public final static double kBalanceThreshold = 5; //In degrees off of upright
        public final static double kOffsetThreshold = 0.25;
        public final static double kBalanceSpeed = 0.027;
        public final static double kOffsetSpeed = 0.027;
        


    }

}
