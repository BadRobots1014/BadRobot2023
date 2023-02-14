// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class ArmSubsystem extends SubsystemBase {

  public final int/*change to motor class later */ baseMotor = 0;
  public final int /*change to motor class later */ forearmMotor = 0;
  private final CANSparkMax m_grabber = new CANSparkMax(ArmConstants.kGrabberPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public int shoulderTicks;
  public int wristTicks;

  public String armPosition = "STORED";

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    m_grabber.setInverted(false); // Find out if needs to be T/F Later
    m_grabber.setIdleMode(IdleMode.kCoast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    this.shoulderTicks = 0;//read ticks from shaft encoder
    this.wristTicks = 0;

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
      case "STORED":
      armPosition = "STORED";
      System.out.println("ARM IS STORED");
      break;
      case "LOW":
      armPosition = "LOW";
      System.out.println("ARM IS LOW");
      break;
      case "MEDIUM":
      armPosition = "MEDIUM";
      System.out.println("ARM IS MEDIUM");
      break;
      case "HIGH":
      armPosition = "HIGH";
      System.out.println("ARM IS HIGH");
      break;
    }
  }

  public void runGrabber(double power){
    m_grabber.set(clampPower(power));
  }

  private static double clampPower(double power) {
    return MathUtil.clamp(power, -1.0, 1.0);
  } 

}
