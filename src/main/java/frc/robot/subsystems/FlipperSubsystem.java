// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FlipperSubsystem extends SubsystemBase {

  private WPI_TalonSRX m_flipper = new WPI_TalonSRX(ShooterConstants.kFlipperPort);

  /** Creates a new FlipperSubsystem. */
  public FlipperSubsystem() {

    m_flipper.setNeutralMode(NeutralMode.Brake);

  }

  public void flip(double power) {
    m_flipper.set(power);
  }

  public void stopFlipper() {
    m_flipper.stopMotor();
  }
}
