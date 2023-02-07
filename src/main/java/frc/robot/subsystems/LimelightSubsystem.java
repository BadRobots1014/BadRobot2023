// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase{
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");


    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");

      /** Creates a new ExampleSubsystem. */
    public LimelightSubsystem() {
        tx.getDouble(0.0);
  
        m_tab.addNumber("LimeLight X", () -> getTableX());
        m_tab.addNumber("LimeLight Y", () -> getTableY());
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    
    public double getTableX() {
        return tx.getDouble(0.0);
    }

    public double getTableY() {
        return ty.getDouble(0.0);
    }
}
