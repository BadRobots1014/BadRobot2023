// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** An example command that uses an example subsystem. */
public class RuntopositionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RuntopositionCommand(ArmSubsystem subsystem) {
    m_armSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("INIT");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("RUNTOPOSITION");
    ArmSubsystem.runToPosition(ArmSubsystem.m_extender, m_armSubsystem.m_extenderEncoder, m_armSubsystem.getCurrArmExtenderEncoderPreset());
    
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