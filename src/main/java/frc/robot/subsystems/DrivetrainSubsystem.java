// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MovementConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

public class DrivetrainSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftA = new CANSparkMax(DriveConstants.kLeftAPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_leftB = new CANSparkMax(DriveConstants.kLeftBPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_rightA = new CANSparkMax(DriveConstants.kRightAPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_rightB = new CANSparkMax(DriveConstants.kRightBPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final DifferentialDrive m_driveTrain = new DifferentialDrive(m_leftA, m_rightA);

    final static ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");
    

    public DrivetrainSubsystem() {

        m_leftA.setInverted(false);
        m_leftB.setInverted(true);
        m_rightA.setInverted(true);
        m_rightB.setInverted(false);


        m_leftA.setIdleMode(IdleMode.kBrake);
        m_leftB.setIdleMode(IdleMode.kBrake);
        m_rightA.setIdleMode(IdleMode.kBrake);
        m_rightB.setIdleMode(IdleMode.kBrake);

        m_leftB.follow(m_leftA);
        m_rightB.follow(m_rightA);

        m_tab.addNumber("Left Power", m_leftA::get);
        m_tab.addNumber("Right Power", m_rightA::get);

        m_tab.add(m_driveTrain.toString(), m_driveTrain);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_driveTrain.tankDrive(clampPower(leftSpeed), clampPower(rightSpeed), true);
    }

    public String getDirection(double leftSpeed, double rightSpeed){

        if (leftSpeed == 0 && rightSpeed == 0){ // If both motors arent moving
            return MovementConstants.kStationary;
        }

        else if (leftSpeed == -rightSpeed){ // If the left motor and right motor are going the exact same speed but in opposite directions
            return MovementConstants.kSpinningInPlace;
        }

        else if (Math.abs(leftSpeed-rightSpeed) < 0.2 && leftSpeed > 0 && rightSpeed > 0){ //If both motors are moving fowards at a similar speed
            return MovementConstants.kForward;
        }

        else if (Math.abs(leftSpeed-rightSpeed) < 0.2 && leftSpeed < 0 && rightSpeed < 0){ //If both motors are moving backwards at a similar speed
            return MovementConstants.kBackward;
        }

        else if (leftSpeed < rightSpeed){//If the left motor is moving backwards faster than the right motor
            return MovementConstants.kTurningCounterclockwise;
        }
        
        else if (leftSpeed > rightSpeed){ // If the right motor is moving backwards faster than the left motor
            return MovementConstants.kTurningClockwise;

        }
        return MovementConstants.kGetDirectionEdgeCase;
    }

    private static double clampPower(double power) {
        return MathUtil.clamp(power, -1.0, 1.0);
    }

    public void stop() {
        m_driveTrain.stopMotor();
    }
}
