// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WinchSubsystem;

/** An example command that uses an example subsystem. */
public class ZeroCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private final WinchSubsystem m_winchSubsystem;
  private DoubleSupplier ZeroTrigger;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroCommand(ArmSubsystem subsystem, WinchSubsystem winch) {
    m_armSubsystem = subsystem;
    m_winchSubsystem = winch;
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
    // m_armSubsystem.m_extender.setIdleMode(IdleMode.kCoast);
    // m_armSubsystem.stopMotor(m_armSubsystem.m_extender);
    // System.out.println("ZERO");
    m_armSubsystem.runExtender(-.3);
    m_winchSubsystem.runWinch(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_armSubsystem.m_extender.setIdleMode(IdleMode.kBrake);
    // m_armSubsystem.resetEncoder(m_armSubsystem.m_extenderEncoder);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
