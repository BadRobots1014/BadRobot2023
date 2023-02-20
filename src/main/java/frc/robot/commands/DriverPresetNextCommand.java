package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriverPresetsSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriverPresetNextCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriverPresetsSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriverPresetNextCommand(DriverPresetsSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  //private boolean is6down = false;
  @Override
  public void execute() 
  {
    
  if (RobotContainer.button6.getAsBoolean())
  {
    DriverPresetsSubsystem.CurrentDriver++;
  }
  /*if (RobotContainer.button7.getAsBoolean())
  {
    DriverPresetsSubsystem.CurrentDriver--;
  }*/
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
