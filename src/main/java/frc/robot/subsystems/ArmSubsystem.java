// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;


public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_winch = new CANSparkMax(ArmConstants.kWinchPort, CANSparkMaxLowLevel.MotorType.kBrushless); // Assume Brushless, unknown currently
  private final CANSparkMax m_extender = new CANSparkMax(ArmConstants.kExtenderPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_grabber = new CANSparkMax(ArmConstants.kGrabberPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public int winchTicks;
  public int extenderTicks;
  public boolean grabber;

  public static String armPosition = ArmConstants.kArmStored;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    m_grabber.setInverted(false); // Find out if needs to be T/F Later
    m_grabber.setIdleMode(IdleMode.kCoast);

    m_winch.setInverted(false); // Find out if needs to be T/F
    m_winch.setIdleMode(IdleMode.kBrake);

    m_extender.setInverted(false); // Find out if needs to be T/F
    m_extender.setIdleMode(IdleMode.kBrake);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    this.winchTicks = 0;//read ticks from shaft encoder
    this.extenderTicks = 0; 
    //code to put arm at preset arm position

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void IK(){
    //inverse kinematics stuff goes here
  }
  public static void setPresetPosition(String armPosition){
    //preset position stuff goes here

    switch (armPosition){
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

  public void closeGrabber(){
    m_grabber.set(ArmConstants.kGrabberPowerR);
  }

  public void openGrabber(){
    m_grabber.set(ArmConstants.kGrabberPowerF);
  }

  public void runGrabber(double power){
    m_grabber.set(clampPower(power));
  }

  public void runGrabberReverse(double power){
    m_grabber.set(-clampPower(power));
  }

  public void stopGrabber(){
    m_grabber.set(ArmConstants.kGrabberBrake);
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

  public void runToPosition(CANSparkMax motor, double pos){
    
  }

}
