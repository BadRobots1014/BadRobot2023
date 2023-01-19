// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberUpCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private final NetworkTable m_datatable;
  //private final StringPublisher Climber;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ClimberUpCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_datatable = inst.getTable("datatable");
   // Climber = m_datatable.getStringTopic("Climber: ").publish();
    //Climber.set("Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.climbUp();
    //Climber.set("Up");
    SmartDashboard.putBoolean("Climber up button: ", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Climber.set("");
    SmartDashboard.putBoolean("Climber up button: ", false);
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
