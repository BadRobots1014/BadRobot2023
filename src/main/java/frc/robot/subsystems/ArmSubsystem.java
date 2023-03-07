// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class ArmSubsystem extends SubsystemBase {

  public final CANSparkMax m_winch = new CANSparkMax(ArmConstants.kWinchPort, CANSparkMaxLowLevel.MotorType.kBrushless); // Assume Brushless, unknown currently
  public static final CANSparkMax m_extender = new CANSparkMax(ArmConstants.kExtenderPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax m_grabber = new CANSparkMax(ArmConstants.kGrabberPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final RelativeEncoder m_extenderEncoder;
 // public final Encoder m_winchEncoder = new Encoder(EncoderConstants.kWinchChannelA, EncoderConstants.kExtenderChannelB);
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Arm");

  public int winchTicks;
  public int extenderTicks;
  public boolean grabber;
  

  public static String armPosition = ArmConstants.kArmStored;
  public static int currArmExtenderEncoderPreset = ArmConstants.kArmStoredPos;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    m_grabber.setInverted(false); // Find out if needs to be T/F Later
    m_grabber.setIdleMode(IdleMode.kCoast);

    m_winch.setInverted(false); // Find out if needs to be T/F
    m_winch.setIdleMode(IdleMode.kBrake);


    m_extender.setInverted(true); //needs to be T

    m_extender.setIdleMode(IdleMode.kBrake);

    m_extenderEncoder = m_extender.getEncoder();
    resetEncoder(m_extenderEncoder);

    m_tab.addString("PresetArmPosition", this::getArmState);
    m_tab.addDouble("Extender Encoder:", this::getExtenderEncoderPosition);
    
    
    //this.setupEncoder(m_extenderEncoder, EncoderConstants.kDefaultDPP, EncoderConstants.kExtenderMinRate, EncoderConstants.kExtenderIsReversed, EncoderConstants.kExtenderSampleSize);

    // this.setupEncoder(m_winchEncoder, EncoderConstants.kDefaultDPP, EncoderConstants.kWinchMinRate, EncoderConstants.kWinchIsReversed, EncoderConstants.kWinchSampleSize);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }

  public void IK(){
    //inverse kinematics stuff goes here
  }
  
  public static void setPresetPosition(String armPos){
    //preset position stuff goes here

    switch (armPos){
      case ArmConstants.kArmStored:
      armPosition = ArmConstants.kArmStored;
      System.out.println("ARM IS STORED");
      break;
      case ArmConstants.kArmLow:
      armPosition = ArmConstants.kArmLow;
      System.out.println("ARM IS LOW");
      break;
      case ArmConstants.kArmMedium:
      armPosition = ArmConstants.kArmMedium;
      System.out.println("ARM IS MEDIUM");
      break;
      case ArmConstants.kArmHigh:
      armPosition = ArmConstants.kArmHigh;
      System.out.println("ARM IS HIGH");
     
      break;
    }
  }

  public void stopMotor(CANSparkMax motor){
    motor.stopMotor();
  }

  public void runGrabber(double power){
    m_grabber.set(clampPower(power));
  }

  public void stopGrabber(){
    m_grabber.stopMotor();
  }

  public void runExtender(double power){
    m_extender.set(clampPower(power));
  }

  public void runWinch(double power){
    m_winch.set(clampPower(power));
  }

  private double clampPower(double power) {
    return MathUtil.clamp(power, -1.0, 1.0);
  }

  public String getArmState(){
    return armPosition;
  } 


  public double getEncoderPosition(RelativeEncoder encoder) {return encoder.getPosition();} // In rotations

  public double getEncoderVelocity(RelativeEncoder encoder) {return encoder.getVelocity();}

  public double getExtenderEncoderPosition(){
    return (getEncoderPosition(m_extenderEncoder));
  }
  
  // public void setupEncoder(Encoder encoder, double distancePerPulse, double minRate, boolean isReversed, int samplesToAverage) {
  //   encoder.reset();
  //   encoder.setDistancePerPulse(distancePerPulse);
  //   encoder.setMinRate(minRate);
  //   encoder.setReverseDirection(isReversed);
  //   encoder.setSamplesToAverage(samplesToAverage);
  // }

  public void resetEncoder(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }

  public static void runToPosition(CANSparkMax motor, RelativeEncoder encoder, double pos){
    double distance = pos - encoder.getPosition();
    double coefficient = MathUtil.clamp(distance, -1.0, 1.0);
    if(pos < encoder.getPosition()){
      motor.set(0.05 * coefficient);
      System.out.println(0.05 * coefficient);
    }else if(pos > encoder.getPosition()){
      motor.set(-0.04 * coefficient);
      System.out.println(-0.04 * coefficient);
    }
  }
}
