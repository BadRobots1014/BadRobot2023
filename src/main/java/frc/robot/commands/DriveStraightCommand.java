// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.GyroConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class DriveStraightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NavXGyroSubsystem m_subsystem;
  private final DrivetrainSubsystem m_drivesubsystem;
  private double initial_yaw;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraightCommand(NavXGyroSubsystem subsystem, DrivetrainSubsystem drivesubsystem) {
    m_subsystem = subsystem;
    m_drivesubsystem = drivesubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, drivesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initial_yaw = m_subsystem.getYaw();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(initial_yaw > m_subsystem.getYaw()){
        m_drivesubsystem.tankDrive(0, 0.3);
    }
    if(initial_yaw < m_subsystem.getYaw()){
        m_drivesubsystem.tankDrive(0.3, 0);
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
