// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;


public class BlinkinSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public BlinkinSubsystem() {
    blinkin.setIdleMode(IdleMode.kBrake);
    gatherer.setIdleMode(IdleMode.kBrake);
  }

  private final CANSparkMax blinkin = new CANSparkMax(BlinkinConstants.kBlinkinPort, MotorType.kBrushless);
  private final CANSparkMax gatherer = new CANSparkMax(7, MotorType.kBrushless);
  // private final SparkMaxAbsoluteEncoder encoder = new SparkMaxAbsoluteEncoder(blinkin, new SparkMaxAbsoluteEncoder.Type(0));

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double pattern) {
    blinkin.set(pattern);
  }

  public void setOcean(double power){
    blinkin.set(power * .25);
  }

  public void setRainbow(){
    blinkin.stopMotor();
  }

  public void setBlue(){
    blinkin.set(0.87);
  }

  public void setGreen(){
    blinkin.set(0.77);
  }

  public void setRed(){
    blinkin.set(0.61);
  }

  public void setWhite(){
    blinkin.set(0.93);
  }

  // public void getEncoder() {
  //   blinkin.getAbsoluteEncoder(EncodingType.k4X);
  // }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
