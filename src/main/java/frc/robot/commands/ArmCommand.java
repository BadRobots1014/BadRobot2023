// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private int m_setArm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem subsystem, int setArm) {
    m_armSubsystem = subsystem;
    m_setArm = setArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmSubsystem.setPresetPosition("STORED");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_setArm){
      case 0:
      ArmSubsystem.armPosition = "STORED";
      break;
      case 1:
      ArmSubsystem.armPosition = "LOW";
      break;
      case 2:
      ArmSubsystem.armPosition = "MEDIUM";
      break;
      case 3:
      ArmSubsystem.armPosition = "HIGH";
      break;

    }
    ArmSubsystem.setPresetPosition(ArmSubsystem.armPosition);
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
