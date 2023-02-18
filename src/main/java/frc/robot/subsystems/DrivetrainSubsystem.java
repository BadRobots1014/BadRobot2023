// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MovementConstants;

public class DrivetrainSubsystem extends SubsystemBase {

    private NavXGyroSubsystem m_gyroSubsystem;

    private final CANSparkMax m_leftFront = new CANSparkMax(DriveConstants.kLeftAPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_leftBack = new CANSparkMax(DriveConstants.kLeftBPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_rightFront = new CANSparkMax(DriveConstants.kRightAPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_rightBack = new CANSparkMax(DriveConstants.kRightBPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final MecanumDrive m_driveTrain = new MecanumDrive(m_leftFront, m_leftBack, m_rightFront, m_rightBack);

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");
    

    public DrivetrainSubsystem( NavXGyroSubsystem gyroSubsystem) {

        m_gyroSubsystem = gyroSubsystem;

        m_leftFront.setInverted(false);
        m_leftBack.setInverted(false);
        m_rightFront.setInverted(false);
        m_rightBack.setInverted(false);

        m_leftFront.setIdleMode(IdleMode.kBrake);
        m_leftBack.setIdleMode(IdleMode.kBrake);
        m_rightFront.setIdleMode(IdleMode.kBrake);
        m_rightBack.setIdleMode(IdleMode.kBrake);

        m_tab.addNumber("Left Front Power", m_leftFront::get);
        m_tab.addNumber("Right Front Power", m_rightFront::get);
        m_tab.addNumber("Left Back Power", m_leftBack::get);
        m_tab.addNumber("Right Back Power", m_rightBack::get);
        m_tab.add(m_driveTrain.toString(), m_driveTrain);
    }

    public void drive(double xSpeed, double ySpeed, double zRotation) {
        Rotation2d gyroYaw = new Rotation2d(m_gyroSubsystem.getYaw());
        m_driveTrain.driveCartesian(xSpeed, ySpeed, zRotation, gyroYaw); //Note that x is forward and y is right. This is fixed when it's initialized in RobotContainer.
    }

    public String getDirection(double leftSpeed, double rightSpeed){

        // if (leftSpeed == 0 && rightSpeed == 0){ // If both motors arent moving
        //     return MovementConstants.kStationary;
        // }

        // else if (rightSpeed == 0){ // If the right motor isnt moving but the left one is(because the above conditional was false)
        //     return MovementConstants.kPivotingOffOfRight;
        // }
        
        // else if (leftSpeed == 0){ // If the left motor isnt moving but the right one is(because the above conditional was false)
        //     return MovementConstants.kPivotingOffOfLeft;
        // }

        // else if (leftSpeed == -rightSpeed){ // If the left motor and right motor are going the exact same speed but in opposite directions
        //     return MovementConstants.kSpinningInPlace;
        // }

        // else if (rightSpeed < 0){ // If the right motor is moving backward
        //     if (leftSpeed == rightSpeed){ // If the left motor is moving backward at the same rate
        //         return MovementConstants.kBackward;
        //     }
        //     else if (leftSpeed < rightSpeed){//If the left motor is moving backwards faster than the right motor
        //         return MovementConstants.kTurningCounterclockwise;
        //     }
        //     else{ // If the right motor is moving backwards faster than the left motor
        //         return MovementConstants.kTurningClockwise;
        //     }
        // }
        
        // else if (rightSpeed > 0){ // If the right motor is moving forwards
        //     if (leftSpeed == rightSpeed){ // If the left motor is moving forwards at the same rate
        //         return MovementConstants.kForward;
        //     }
        //     else if (leftSpeed < rightSpeed){ // If the right motor is going forward faster than the left motor
        //         return MovementConstants.kTurningCounterclockwise;
        //     }
        //     else{ // If the left motor is going forward faster than the right motor
        //         return MovementConstants.kTurningClockwise;
        //     }
        // }
        return MovementConstants.kGetDirectionEdgeCase;
    }

    private static double clampPower(double power) {
        return MathUtil.clamp(power, -1.0, 1.0);
    }

    public void stop() {
        m_driveTrain.stopMotor();
    }
}
