// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** An example command that uses an example subsystem. */
public class ZAxisArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZAxisArmCommand(ArmSubsystem subsystem) {
    m_armSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // System.out.println("INIT");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmSubsystem.runToPosition(ArmSubsystem.m_extender, m_armSubsystem.m_extenderEncoder, (m_armSubsystem.getLeftZAxis() * m_armSubsystem.getExtenderUpperBound()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.setDunkState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
