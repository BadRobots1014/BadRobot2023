package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorSubsystem;

public class ColorSensorCommand extends CommandBase
{
    private ColorSensorSubsystem m_subsystem;

    public ColorSensorCommand(ColorSensorSubsystem subsystem)
    {
        addRequirements(subsystem);
        m_subsystem = subsystem;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        m_subsystem.getColor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}