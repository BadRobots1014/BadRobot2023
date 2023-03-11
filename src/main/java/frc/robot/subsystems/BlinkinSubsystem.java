// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class BlinkinSubsystem extends SubsystemBase {

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Blinkin");

  private final Spark blinkin;

  public BlinkinSubsystem() {
    blinkin = new Spark(BlinkinConstants.kBlinkinPort);
    m_tab.add(blinkin.toString(), blinkin);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double pattern) {
    blinkin.set(pattern);
  }

  public void setOcean(){
    blinkin.set(-0.95);
  }

  public void setRainbow(){
    blinkin.set(-0.99);
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
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
