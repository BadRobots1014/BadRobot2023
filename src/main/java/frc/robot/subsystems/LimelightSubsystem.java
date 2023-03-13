// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.function.Supplier;

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
    //public LimelightSubsystem() {
    public LimelightSubsystem() {
        //tx.getDouble(0.0);


  
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
    
    public void setPipeline(int pipelineNum) {
        table.getEntry("pipeline").setNumber(pipelineNum);
    }

    public double getTableX() {
        return tx.getDouble(0.0);
    }

    public double getPIDTurnTableX()
    {
        if(Math.abs(getTableX()) < LimelightConstants.kAngleDeadZone)
            return 0.0;

        return getTableX() / LimelightConstants.kMaxSpeedAngle;  // 30 degrees is the extremity of either side
    }

    public double getTableY() {
        return ty.getDouble(0.0);
    }

    
    public double getAngleToTargetDeg(double llMADeg) {
        return getTableY() + llMADeg;
    }
    public double getAngleToTargetDegInverse(double llMADeg) {
        return -1 * getTableY() + llMADeg;
    }

    public double calculateDistanceInch(double llMADeg, double llMH, double tH) {
        // inverse the angle because the camera is tipped downwards
        // might have to subtract the height.
        return (llMH - tH) / Math.tan(Math.toRadians(getAngleToTargetDegInverse(llMADeg)));
    }

    public double calculateDistanceInchPID(double llMADeg, double llMH, double tH) {
        // subtracting the distance target from the calculated distance allows us to 
        // find the offset from the target

        // deadzoning
        if(Math.abs((calculateDistanceInch(llMADeg, llMH, tH) - LimelightConstants.kDistTarget)) < LimelightConstants.kDistDeadZone)
            return 0;

        // dividing the distance by the max offset
        // this is done in order to make the robot compensate at max speed at the max offset, 
        // and prevent the control effort from going over 1 in the range of -MaxOffset to MaxOffset
        return (calculateDistanceInch(llMADeg, llMH, tH) - LimelightConstants.kDistTarget) / LimelightConstants.kMaxSpeedDistOffset;
    }
    
    public double getTableA() {
        return ta.getDouble(0.0);
    }
}
