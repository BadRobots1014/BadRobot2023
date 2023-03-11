// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.GrabberSubsystem;

/** An example command that uses an example subsystem. */
public class DunkCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private double dunkPos;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DunkCommand(ArmSubsystem subsystem) {
    m_armSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   dunkPos = -1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (dunkPos == -1000) dunkPos = m_armSubsystem.getExtenderEncoderPosition();
    m_armSubsystem.runToPosition(m_armSubsystem.m_extender, m_armSubsystem.m_extenderEncoder, dunkPos - 1, .1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dunkPos = -1000;
    // m_armSubsystem.setDunkState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
