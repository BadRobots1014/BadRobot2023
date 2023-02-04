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
    private final CANSparkMax m_left = new CANSparkMax(DriveConstants.kLeftPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_right = new CANSparkMax(DriveConstants.kRightPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final DifferentialDrive m_driveTrain = new DifferentialDrive(m_left, m_right);

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");

    public DrivetrainSubsystem() {
        m_left.setInverted(false);
        m_right.setInverted(true);
        m_right.set(0);

        m_left.setIdleMode(IdleMode.kBrake);
        m_right.setIdleMode(IdleMode.kBrake);

        m_tab.addNumber("Left Power", m_left::get);
        m_tab.addNumber("Right Power", m_right::get);
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

        else if (Math.abs(leftSpeed-rightSpeed) < 0.2){ // If the left motor is moving backward at about the same rate
            if(leftSpeed > 0 && rightSpeed > 0) return MovementConstants.kForward;
            if(leftSpeed < 0 && rightSpeed < 0) return MovementConstants.kBackward;
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
