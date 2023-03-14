// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class RuntopositionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;

  private final double m_position;
  private final double m_speed;

  private final double m_winchPosition;
  private final double m_winchSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RuntopositionCommand(ArmSubsystem subsystem, double position, double speed, double winchPosition, double winchSpeed) {
    m_armSubsystem = subsystem;
    m_position = position;
    m_speed = speed;
    m_winchPosition = winchPosition;
    m_winchSpeed = winchSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.runToPosition(m_armSubsystem.m_extender, m_armSubsystem.m_extenderEncoder, m_position, m_speed);
    // m_armSubsystem.runToPosition(m_armSubsystem.m_winch, m_armSubsystem.m_winchEncoder, m_winchPosition, m_winchSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.runExtender(0);
    m_armSubsystem.stopMotor(ArmSubsystem.m_extender);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
