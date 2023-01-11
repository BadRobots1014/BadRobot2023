// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonSRX m_left = new TalonSRX(ClimberConstants.kLeftClimberPort);
  private final TalonSRX m_right = new TalonSRX(ClimberConstants.kRightClimberPort);

  public ClimberSubsystem() {}

  public void climbUp() {
    //Sets motor power to climb speed
    m_left.set(ControlMode.PercentOutput, ClimberConstants.climberUpSpeed);
    m_right.set(ControlMode.PercentOutput, ClimberConstants.climberUpSpeed);
  }

  public void climbDown() {
    //Sets motor power to negative climb speed
    /*Code here*/
  }

  public void stop() {
    //Sets motor power to 0
    m_left.set(ControlMode.PercentOutput, 0);
    m_right.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
  }
}
