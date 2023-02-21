// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class GrabberCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private String m_mode;
  private DoubleSupplier m_power;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GrabberCommand(ArmSubsystem subsystem, String runMode, DoubleSupplier trigger) {
    m_subsystem = subsystem;
    m_mode = runMode;
    m_power = trigger;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (m_mode) {
      case (ArmConstants.kManualRunForward):
        m_subsystem.runGrabber(m_power.getAsDouble());
        break;
      case (ArmConstants.kManualRunBackward):
        m_subsystem.runGrabberReverse(m_power.getAsDouble());
        break;
      case (ArmConstants.kPresetRunForward):
        m_subsystem.openGrabber();
        break;
      case (ArmConstants.kPresetRunBackward):
        m_subsystem.closeGrabber();
        break;
      case (ArmConstants.kBrake):
        m_subsystem.stopGrabber();
        break;
      default:
        m_subsystem.stopGrabber();
        break;
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
