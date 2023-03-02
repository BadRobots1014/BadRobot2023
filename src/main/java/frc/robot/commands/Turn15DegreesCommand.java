// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.NavXGyroSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

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
    
    DrivetrainSubsystem.m_tab.addBoolean("4 button", RobotContainer.turnLeft15Button::getAsBoolean);
    DrivetrainSubsystem.m_tab.addBoolean("5 button", RobotContainer.turnRight15Button::getAsBoolean);

    this.rightSpeed = rightSpeed;
    this.leftSpeed = leftSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  boolean rightDown = false;
  boolean leftDown = false;

  @Override
  public void execute() {
    System.out.println("Being run");
    currentYaw = m_subsystem.getYaw();
    if (!RobotContainer.turnLeft15Button.getAsBoolean() && leftDown)
    {
        turn(true);
        leftDown = false;
    }
    if (RobotContainer.turnLeft15Button.getAsBoolean())
    {
       leftDown = true;
    }

    if (!RobotContainer.turnRight15Button.getAsBoolean() && rightDown)
    {
      turn(false);
      rightDown = false;
    }
    if (RobotContainer.turnRight15Button.getAsBoolean())
    {
      rightDown = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void turn(boolean left)
  {
    double previousyaw = currentYaw;
    if (left)
    {
      while (previousyaw - 15 > m_subsystem.getYaw());
      {
        m_DrivetrainSubsystem.tankDrive(.5, -.5);
        System.out.println("Is doing it");
      }
    }
    else
    {
      while (previousyaw + 15 > m_subsystem.getYaw());
      {
        m_DrivetrainSubsystem.tankDrive(-.5, .5);
        System.out.println("Is doing it right");
      }
    }
  }
}
