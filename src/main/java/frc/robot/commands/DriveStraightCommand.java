// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.GyroConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class DriveStraightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NavXGyroSubsystem m_subsystem;
  private final DrivetrainSubsystem m_drivesubsystem;
  private double initial_yaw;
  private DoubleSupplier m_leftSpeed;
  private DoubleSupplier m_rightSpeed;
  private DoubleSupplier m_throttle;
  private Double m_driveSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraightCommand(NavXGyroSubsystem subsystem, DrivetrainSubsystem drivesubsystem, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DoubleSupplier throttle) {
    m_subsystem = subsystem;
    m_drivesubsystem = drivesubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, drivesubsystem);

    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    m_throttle = throttle;
    m_driveSpeed = Math.max(Math.abs(m_leftSpeed.getAsDouble()), Math.abs(m_rightSpeed.getAsDouble()))*m_throttle.getAsDouble();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.reset();
    initial_yaw = m_subsystem.getYaw();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivesubsystem.tankDrive(m_driveSpeed, m_driveSpeed);

    double angle = initial_yaw - m_subsystem.getYaw();
    if(angle >= GyroConstants.kOffsetThreshold) {
        // the formula that Noirit used, condensed down (even more now)
        double speed = angle * GyroConstants.kOffsetSpeed;
        m_drivesubsystem.tankDrive(m_driveSpeed,m_driveSpeed+speed);
    }
    if(angle <= -1 * GyroConstants.kOffsetThreshold) {
        // the formula that Noirit used, condensed down (even more now)
        double speed = angle * GyroConstants.kOffsetSpeed;
        m_drivesubsystem.tankDrive(m_driveSpeed+speed,m_driveSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
