package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorSubsystem;

public class ColorSensorCommand extends CommandBase
{
    private ColorSensorSubsystem m_subsystem;
    private Supplier<String> Color = () -> m_subsystem.getColorAsString();

    public ColorSensorCommand(ColorSensorSubsystem subsystem)
    {
        addRequirements(subsystem);
        m_subsystem = subsystem;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
        m_subsystem.m_tab.addString("SensorColor", Color);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        m_subsystem.read();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
        m_subsystem.close();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}