// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private WPI_TalonSRX m_front = new WPI_TalonSRX(ShooterConstants.kFrontPort);
  private WPI_TalonSRX m_back = new WPI_TalonSRX(ShooterConstants.kBackPort);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    m_front.setNeutralMode(NeutralMode.Coast);
    m_back.setNeutralMode(NeutralMode.Coast);

    m_front.setInverted(false);
    m_back.setInverted(false);

    m_back.follow(m_front);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setFlywheelSpeed(double speed) {
    m_front.set(speed);
  }

  public void stopFlywheel() {
    m_front.stopMotor();
  }
}
