// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicScrollPaneUI.HSBChangeListener;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EncoderConstants;


public class GrabberSubsystem extends SubsystemBase {

 
  public static final CANSparkMax m_grabber = new CANSparkMax(ArmConstants.kGrabberPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Grabber");
  public boolean grabber;
  private double m_currentAmps;
  

  /** Creates a new ExampleSubsystem. */
  public GrabberSubsystem() {
    m_grabber.setInverted(false); // Find out if needs to be T/F Later
    m_grabber.setIdleMode(IdleMode.kBrake);

    m_tab.addNumber("Grabber Amp Output: ", this::getCurrent);
  }  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_currentAmps = getCurrent();
    System.out.println(m_currentAmps);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  } 

  public void runGrabber(double power){
    m_grabber.set(clampPower(power));
  }

  public void stopGrabber(){
    m_grabber.stopMotor();
  }

  public double getCurrent(){
    return m_grabber.getOutputCurrent();
  }


  private double clampPower(double power) {
    return MathUtil.clamp(power, -1.0, 1.0);
  }

}
