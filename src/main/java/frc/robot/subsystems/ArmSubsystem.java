// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class ArmSubsystem extends SubsystemBase {

  public final CANSparkMax m_winch = new CANSparkMax(ArmConstants.kWinchPort, CANSparkMaxLowLevel.MotorType.kBrushless); // Assume Brushless, unknown currently
  public static final CANSparkMax m_extender = new CANSparkMax(ArmConstants.kExtenderPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final GrabberSubsystem m_GrabberSubsystem = new GrabberSubsystem();
  public final RelativeEncoder m_extenderEncoder;
  
 // public final Encoder m_winchEncoder = new Encoder(EncoderConstants.kWinchChannelA, EncoderConstants.kExtenderChannelB);
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Arm");

  public int winchTicks;
  public int extenderTicks;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {

  m_winch.setInverted(false); // Find out if needs to be T/F
  m_winch.setIdleMode(IdleMode.kBrake);


  m_extender.setInverted(true); //needs to be T

  m_extender.setIdleMode(IdleMode.kBrake);

  m_extenderEncoder = m_extender.getEncoder();
  resetEncoder(m_extenderEncoder);

    m_tab.addDouble("Extender Encoder:", this::getExtenderEncoderPosition);
    m_tab.addString("Arm Preset Position", this::getArmPresetLocation);
    m_tab.addNumber("Grabber Amp Output: ", m_GrabberSubsystem::getCurrent);
    m_tab.addString("Grabber State: ", m_GrabberSubsystem::getGrabberState);
    m_tab.addBoolean("Grabber Filled?", m_GrabberSubsystem::isGrabberFilled);
  

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


  public void runExtender(double power){
    m_extender.set(clampPower(power));
  }

  public void runWinch(double m_winchUp){
    m_winch.set(clampPower(m_winchUp));
  }

  private static double clampPower(double power) {
    return MathUtil.clamp(power, -1.0, 1.0);
  }

  public double getEncoderPosition(RelativeEncoder encoder) {return encoder.getPosition();} // In rotations

  public double getEncoderVelocity(RelativeEncoder encoder) {return encoder.getVelocity();}

  public double getExtenderEncoderPosition(){
    return (getEncoderPosition(m_extenderEncoder));
  }

  public String getArmPresetLocation(){
    if(Math.abs(getExtenderEncoderPosition() - ArmConstants.kArmHighPos) <= 1){
      return ArmConstants.kArmHigh;
    }
    else if(Math.abs(getExtenderEncoderPosition() - ArmConstants.kArmMediumPos) <= 1){
      return ArmConstants.kArmMedium;
    }
    else if(Math.abs(getExtenderEncoderPosition() - ArmConstants.kArmLowPos) <= 1){
      return ArmConstants.kArmLow;
    }
    else if(Math.abs(getExtenderEncoderPosition() - ArmConstants.kArmStoredPos) <= 1){
      //return "WEEE";
      return ArmConstants.kArmStored;
    }
    else {
      return ArmConstants.kArmManual;
    }
    

     
  }

  public double getExtenderUpperBound(){
    return ArmConstants.kMaxHeight;
  }

  public void resetEncoder(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }

  public static void runToPosition(CANSparkMax motor, RelativeEncoder encoder, double pos, double speed){
    
    double dis = pos - encoder.getPosition();
    if(dis < 0){
      speed = 0.12;
    }
    double motorspeed = clampPower(dis) * speed;
    motor.set(motorspeed);

  }
}
