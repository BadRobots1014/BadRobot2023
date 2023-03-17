// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;

/** An example command that uses an example subsystem. */
public class ArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;

  private final double m_power;
  private final DoubleSupplier m_powerSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem subsystem, double power) {
    m_armSubsystem = subsystem;
    m_power = power;
    m_powerSupplier = new DoubleSupplier() {public double getAsDouble() {return m_power;}};
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public ArmCommand(ArmSubsystem subsystem, DoubleSupplier power) {
    m_armSubsystem = subsystem;
    m_powerSupplier = power;
    m_power = m_powerSupplier.getAsDouble();

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((m_armSubsystem.getExtenderEncoderPosition() < ArmConstants.kMaxHeight && m_powerSupplier.getAsDouble() > 0) || (m_armSubsystem.getExtenderEncoderPosition() > ArmConstants.kMinHeight && m_powerSupplier.getAsDouble() < 0)){
      m_armSubsystem.runExtender(m_powerSupplier.getAsDouble());
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.runExtender(0);
    m_armSubsystem.stopMotor(m_armSubsystem.m_extender);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
