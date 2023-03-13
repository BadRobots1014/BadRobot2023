// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.GyroConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class BalanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NavXGyroSubsystem m_subsystem;
  private final DrivetrainSubsystem m_drivesubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BalanceCommand(NavXGyroSubsystem subsystem, DrivetrainSubsystem drivesubsystem) {
    m_subsystem = subsystem;
    m_drivesubsystem = drivesubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, drivesubsystem);
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_subsystem.getRoll();
    double speed;
    if(Math.abs(angle) >= GyroConstants.kBalanceThreshold) {
        // the formula that Noirit used, condensed down (even more now)
        speed = angle * GyroConstants.kBalanceSpeed;
        m_drivesubsystem.tankDrive(speed, speed);
    }
    else {
        m_drivesubsystem.stop();
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
}

