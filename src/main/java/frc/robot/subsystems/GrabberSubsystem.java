// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class GrabberSubsystem extends SubsystemBase {

 
  private static final CANSparkMax m_grabber = new CANSparkMax(ArmConstants.kGrabberPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Grabber");
  private double m_currentAmps;
  private String m_grabberState = ArmConstants.kGrabberEmpty;
  

  /** Creates a new ExampleSubsystem. */
  public GrabberSubsystem() {
    m_grabber.setInverted(false); // Find out if needs to be T/F Later
    m_grabber.setIdleMode(IdleMode.kBrake);

    m_tab.addNumber("Grabber Amp Output: ", this::getCurrent);
    m_tab.addString("Grabber State: ", this::getGrabberState);
  }  
  @Override
  public void periodic() {
    m_currentAmps = getCurrent();
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
    if (m_currentAmps >= ArmConstants.kGrabberAmpMax){
      m_grabberState = ArmConstants.kGrabberFilled;
    } else if (m_currentAmps < ArmConstants.kGrabberAmpMax && m_currentAmps > 1){
      m_grabberState = ArmConstants.kGrabberEmpty;
    }

    return m_grabber.getOutputCurrent();
  }

  public String getGrabberState(){
    return m_grabberState;
  }

  private double clampPower(double power) {
    return MathUtil.clamp(power, -1.0, 1.0);
  }

}
