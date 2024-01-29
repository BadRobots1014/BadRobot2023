package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem {
    private ShuffleboardTab m_limelightTab = Shuffleboard.getTab("Limelight");;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    DoubleSupplier LimelightX = new DoubleSupplier() {
            public double getAsDouble()
            {
                return tx.getDouble(0.0);
            }};
    DoubleSupplier LimelightY = new DoubleSupplier() {
            public double getAsDouble()
            {
                return ty.getDouble(0.0);
            }};
    DoubleSupplier LimelightA = new DoubleSupplier() {
            public double getAsDouble()
            {
                return ta.getDouble(0.0);
            }};

    //read values periodically

    public LimelightSubsystem()
    {
        m_limelightTab.addDouble("LimelightX", LimelightX);
        m_limelightTab.addDouble("LimelightY", LimelightY);
        m_limelightTab.addDouble("LimelightA", LimelightA);
    }
}

// Testing...