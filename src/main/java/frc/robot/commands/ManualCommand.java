// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ManualCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private double armTicks;
  private boolean armUp;
  private boolean firstLoop;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualCommand(ArmSubsystem subsystem, boolean up) {
    m_armSubsystem = subsystem;
    armTicks = m_armSubsystem.getEncoderPosition(m_armSubsystem.m_extenderEncoder);
    armUp = up;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("INIT");
    firstLoop = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armUp && armTicks <= 39 && firstLoop){
      System.out.println("UP");
      armTicks += 4;
    }
    if(!armUp && armTicks >= 2 && firstLoop){
      System.out.println("DOWN");
      armTicks = armTicks - 4;
    }
    System.out.println(armTicks);
    ArmSubsystem.runToPosition(ArmSubsystem.m_extender, m_armSubsystem.m_extenderEncoder, armTicks, 0.2);
    firstLoop = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
