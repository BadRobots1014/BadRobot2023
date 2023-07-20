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
    setDefault();
  }

  public void setBlinkin(double pattern) {
    blinkin.set(pattern);
  }

  public void setDefault(){
    setBlinkin(.09);
  }

  public void setSpinUp() {
    setBlinkin(-.11);
  }

  public void setShoot() {
    setBlinkin(.61);
  }

  public void setOcean(){
    setBlinkin(-.95);
  }

  public void setRainbow(){
    setBlinkin(-.99);
  }

  public void setBlue(){
    setBlinkin(.87);
  }

  public void setGreen(){
    setBlinkin(.77);
  }

  public void setRed(){
    setBlinkin(.61);
  }

  public void setWhite(){
    setBlinkin(.93);
  }

}
