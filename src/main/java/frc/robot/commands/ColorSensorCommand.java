package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorSubsystem;

public class ColorSensorCommand extends CommandBase
{
    private int ColorSensorStatus = 0;
    private ColorSensorSubsystem m_subsystem;
    public ColorSensorCommand(ColorSensorSubsystem subsystem)
    {
        addRequirements(subsystem);
        m_subsystem = subsystem;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
        m_subsystem.read();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        ColorSensorStatus = m_subsystem.status();
        
        if (ColorSensorStatus == m_subsystem.blue)
        {
            
        }
        if (ColorSensorStatus == m_subsystem.red)
        {

        }
        if (ColorSensorStatus == m_subsystem.green)
        {

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}