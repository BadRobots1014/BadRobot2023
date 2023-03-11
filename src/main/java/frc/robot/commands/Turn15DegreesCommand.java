// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.NavXGyroSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Turn15DegreesCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NavXGyroSubsystem m_subsystem;
  private final DrivetrainSubsystem m_DrivetrainSubsystem;

  private double currentYaw = 0;

  DoubleSupplier rightSpeed;
  DoubleSupplier leftSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Turn15DegreesCommand(NavXGyroSubsystem gyroSubsystem, DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier rightSpeed, DoubleSupplier leftSpeed) {
    m_subsystem = gyroSubsystem;
    m_DrivetrainSubsystem = drivetrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gyroSubsystem);

    DrivetrainSubsystem.m_tab.addDouble("Yaw", gyroSubsystem::getYaw);
    DrivetrainSubsystem.m_tab.addBoolean("turn command", this::isRunning);
    DrivetrainSubsystem.m_tab.addBoolean("Is turning", this::isTurning);
    DrivetrainSubsystem.m_tab.addDouble("current yaw", this::getTargetYaw);

    this.rightSpeed = rightSpeed;
    this.leftSpeed = leftSpeed;
  }

  boolean left = false;
  boolean turning = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Command Run");

    hasrun = true;
    
    currentYaw = m_subsystem.getYaw();
    
    if (RobotContainer.turnLeft15Button.getAsBoolean())
    {
      left = true;
      turning = true;
    }

    if (RobotContainer.turnRight15Button.getAsBoolean())
    {
      left = false;
      turning = true;
    }    
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  private boolean hasrun = false;

  @Override
  public void execute() {

    if (left)
    {
      m_DrivetrainSubsystem.tankDrive(-.5, .5);
    }
    else
    {
      m_DrivetrainSubsystem.tankDrive(.25, -.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hasrun = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (left)
    {
      if (currentYaw - 15 > m_subsystem.getYaw())
      {
        System.out.println("returns true left");
        return true;
      }
    }
    else
    {
      if (currentYaw + 15 < m_subsystem.getYaw())
      {
        System.out.println("Return right");
        return true;
      }
    }
    turning = false;
    return false;
  }

  public boolean isRunning()
  {
    return hasrun;
  }

  public boolean isTurning()
  {
    return turning;
  }

  public double getTargetYaw()
  {
    if (left)
    {
      return currentYaw - 15;
    }
    else{
      return currentYaw + 15;
    }
    
  }
}
