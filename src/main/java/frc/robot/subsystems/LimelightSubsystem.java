// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Limelight");
  
  public LimelightSubsystem() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(2);
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    

//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);

//NetworkTableEntry AprilTagID = table.getEntry("tid");

System.out.println("LIMELIGHT SUBSYSTEM");
  m_tab.addNumber("AprilTag ID: " + this.toString(), this::getAprilTagID);
  m_tab.addNumber("Tag tx", this::getTx);
  m_tab.addNumber("Tag ty", this::getTy);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public double getAprilTagID(){
    NetworkTableEntry tID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid");
    double AprilTagID = tID.getDouble(0.0);
    return AprilTagID;
  }

  public double getTx(){
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    return tx;
  }

  public double getTy(){
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    return ty;
  }

  public double getTa(){
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
    return ta;
  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}