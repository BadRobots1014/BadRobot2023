// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Limelight");
  GenericEntry customTagHeight;
  
  public LimelightSubsystem() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(2);
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");
    System.out.println("LIMELIGHT SUBSYSTEM");
    m_tab.addNumber("AprilTag ID: ", this::getAprilTagID);
    m_tab.addNumber("Tag tx", this::getTx);
    m_tab.addNumber("Tag ty", this::getTy);

    m_tab.add("Custom April Tag Height", 0);
    customTagHeight = m_tab.add("Custom April Tag Height", 0).getEntry();
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

  public double getTagHeight(){ //returns height of the viewed april tag in inches based off of their expected game field height
    double customHeight = customTagHeight.getDouble(0);
    if (customHeight != 0) return customHeight;
    double id = getAprilTagID();

    if (id == 1 || id == 2 || id == 5 || id == 6 || id == 9 || tag == 10){
      return 53.38; //height of source and amp april tags
    }
    if (id == 3 || id == 4 || id == 5 || id == 7){
      return 57.13;
    }
    else {
      return 52.00; //remaining tags are all stage april tags
    }
  }

  public double getDistance(){
    double d = (getTagHeight() - camHeight) / Math.tan(getTy()+camAngle);
    return d;
  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}