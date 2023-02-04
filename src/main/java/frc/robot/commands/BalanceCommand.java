// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class BalanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NavXGyroSubsystem m_subsystem;
  private final BlinkinSubsystem m_ledsubsystem;
  private final DrivetrainSubsystem m_drivesubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BalanceCommand(NavXGyroSubsystem subsystem, BlinkinSubsystem lightSubsystem, DrivetrainSubsystem drivesubsystem) {
    m_subsystem = subsystem;
    m_ledsubsystem = lightSubsystem;
    m_drivesubsystem = drivesubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, lightSubsystem, drivesubsystem);
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
    double speed = 0;
    if(angle >= 5 && angle < 10){
        speed = 0.10;
    }
    else if(angle >= 10 && angle < 15){
        speed = 0.15;
        m_drivesubsystem.tankDrive(speed, speed);
        m_ledsubsystem.setRed();
    }
    else if(angle >= 15 && angle < 20){
        speed = 0.20;
        m_drivesubsystem.tankDrive(speed, speed);
        m_ledsubsystem.setRed();
    }
    else if(angle >= 20 && angle < 25){
        speed = 0.25;
        m_drivesubsystem.tankDrive(speed, speed);
        m_ledsubsystem.setRed();
    }
    else if(angle >= 25 && angle < 30){
        speed = 0.30;
        m_drivesubsystem.tankDrive(speed, speed);
        m_ledsubsystem.setRed();
    }
    else if(angle >= 35 && angle < 40){
        speed = 0.35;
        m_drivesubsystem.tankDrive(speed, speed);
        m_ledsubsystem.setRed();
    }
    else if(m_subsystem.getRoll() < -25){
        m_drivesubsystem.tankDrive(-0.5 * 0.5, -0.5 * 0.5);
        m_ledsubsystem.setRed();
    }
    else{
        m_drivesubsystem.stop();
        m_ledsubsystem.setGreen();
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

