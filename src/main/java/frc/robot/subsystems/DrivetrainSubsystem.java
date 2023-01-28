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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DrivetrainSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_left = new WPI_TalonSRX(DriveConstants.kLeftPort);
    private final WPI_TalonSRX m_left2 = new WPI_TalonSRX(DriveConstants.kSecondLeftPort);
    private final WPI_TalonSRX m_right = new WPI_TalonSRX(DriveConstants.kRightPort);
    private final WPI_TalonSRX m_right2 = new WPI_TalonSRX(DriveConstants.kSecondRightPort);

    private final DifferentialDrive m_driveTrain;

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");

    public DrivetrainSubsystem() {
        m_left.setInverted(true);
        m_left2.setInverted(true);
        m_right.setInverted(false);
        m_right2.setInverted(false);

        m_left.setNeutralMode(NeutralMode.Brake);
        m_right.setNeutralMode(NeutralMode.Brake);
        m_left2.setNeutralMode(NeutralMode.Brake);
        m_right2.setNeutralMode(NeutralMode.Brake);

        m_left2.follow(m_left);
        m_right2.follow(m_right);

        m_tab.addNumber("Left Power", m_left::get);
        m_tab.addNumber("Right Power", m_right::get);

        m_driveTrain = new DifferentialDrive(m_left, m_right);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_driveTrain.tankDrive(clampPower(leftSpeed), clampPower(rightSpeed), true);
    }

    private static double clampPower(double power) {
        return MathUtil.clamp(power, -1.0, 1.0);
    }

    public void stop() {
        m_driveTrain.stopMotor();
    }
}
