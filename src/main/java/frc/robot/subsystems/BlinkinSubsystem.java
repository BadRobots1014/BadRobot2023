// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BlinkinSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public BlinkinSubsystem() {}

  private final Spark revSpark = new Spark(0);

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOcean(){
    revSpark.set(-0.95);
  }

  public void setRainbow(){
    revSpark.set(-0.99);
  }

  public void setBlue(){
    revSpark.set(0.87);
  }

  public void setGreen(){
    revSpark.set(0.77);
  }

  public void setRed(){
    revSpark.set(0.61);
  }

  public void setWhite(){
    revSpark.set(0.93);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
