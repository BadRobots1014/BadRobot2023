// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class WinchSubsystem extends SubsystemBase {

  public final CANSparkMax m_winch = new CANSparkMax(ArmConstants.kWinchPort, CANSparkMaxLowLevel.MotorType.kBrushed);

  private SparkMaxLimitSwitch m_winch_forwardLimit;
  private SparkMaxLimitSwitch m_winch_reverseLimit;
  
 // public final Encoder m_winchEncoder = new Encoder(EncoderConstants.kWinchChannelA, EncoderConstants.kExtenderChannelB);
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Arm");

  public int winchTicks;

  /** Creates a new ExampleSubsystem. */
  public WinchSubsystem() {

  m_winch.setInverted(true);
  m_winch.setIdleMode(IdleMode.kBrake);
  m_winch_forwardLimit = m_winch.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  m_winch_reverseLimit = m_winch.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }

  public void stopMotor(CANSparkMax motor){
    motor.stopMotor();
  }

  public void runWinch(double m_winchUp){
    m_winch.set(clampPower(m_winchUp));
    System.out.println("Winch up " + m_winchUp);
  }

  private static double clampPower(double power) {
    return MathUtil.clamp(power, -1.0, 1.0);
  }

  
  public double getExtenderUpperBound(){
    return ArmConstants.kMaxHeight;
  }
}
